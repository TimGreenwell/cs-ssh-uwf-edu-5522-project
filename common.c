#define _POSIX_C_SOURCE 200809L
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "common.h"
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <ctype.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <time.h>
#include <stdio.h>

/* These are implemented in the drivers; declare so we can call them here. */
double full_haversine(double lat1,double lon1,double lat2,double lon2);
double distance_from_half(double a);

#ifndef MAX_LINE
#define MAX_LINE 2048
#endif
#ifndef INIT_CAPACITY
#define INIT_CAPACITY 256
#endif

/* -------- timing / fs ---------- */
double wall_time(void){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int ensure_dir(const char *path){
    struct stat st;
    if (stat(path, &st) == 0) {
        if (S_ISDIR(st.st_mode)) return 0;
        errno = ENOTDIR;
        return -1;
    }
    if (mkdir(path, 0755) == 0) return 0;
    if (errno == EEXIST) return 0;
    return -1;
}

/* -------- strings ---------- */
void rstrip_newline(char *s){
    if(!s) return;
    size_t n = strlen(s);
    while(n && (s[n-1]=='\n' || s[n-1]=='\r')) s[--n] = '\0';
}

void rtrim_spaces(char *s){
    if(!s) return;
    size_t n = strlen(s);
    while(n && isspace((unsigned char)s[n-1])) s[--n] = '\0';
}

int split_csv_simple(char *line, char *fields[], int max_fields){
    int n = 0;
    char *p = line, *start = line;
    while(*p && n < max_fields){
        if(*p == ','){
            *p = '\0';
            fields[n++] = start;
            start = p + 1;
        }
        p++;
    }
    if(n < max_fields) fields[n++] = start;
    return n;
}

bool parse_double_strict(const char *s, double *out){
    if(!s || !*s) return false;
    errno = 0;
    char *end = NULL;
    double v = strtod(s, &end);
    if(errno || end == s || *end != '\0' || !isfinite(v)) return false;
    *out = v;
    return true;
}

double parse_double_or0(const char *s){
    if(!s || !*s) return 0.0;
    char *end = NULL;
    double v = strtod(s, &end);
    return (end && *end == '\0') ? v : 0.0;
}

const char* dash_if_empty(const char *s){
    return (s && *s) ? s : "-";
}

/* -------- CSV load / print / logs ---------- */
int load_csv(const char *filename, Aircraft **arr, int *count, int *capacity){
    FILE *fp = fopen(filename,"r");
    if(!fp){ perror("CSV open"); return -1; }

    if ((*arr == NULL) || (*capacity <= 0)) {
        *capacity = (*capacity > 0) ? *capacity : INIT_CAPACITY;
        *arr = (Aircraft*)malloc((size_t)(*capacity) * sizeof(Aircraft));
        if(!*arr){ perror("malloc"); fclose(fp); return -1; }
        *count = 0;
    }

    char line[MAX_LINE];

    int idx_time=-1, idx_icao=-1, idx_lat=-1, idx_lon=-1,
        idx_vel=-1, idx_head=-1, idx_callsign=-1, idx_baro=-1;

    if (!fgets(line, MAX_LINE, fp)){ fclose(fp); return 0; }
    rstrip_newline(line);

    {
        char *fields[64];
        int nf = split_csv_simple(line, fields, 64);

        /* Skip our own output files (rank header) */
        if (nf > 0 && !strcmp(fields[0], "rank")) { fclose(fp); return 0; }

        for (int i=0;i<nf;i++){
            if      (!strcmp(fields[i],"time"))          idx_time = i;
            else if (!strcmp(fields[i],"icao24"))        idx_icao = i;
            else if (!strcmp(fields[i],"lat"))           idx_lat  = i;
            else if (!strcmp(fields[i],"lon"))           idx_lon  = i;
            else if (!strcmp(fields[i],"velocity"))      idx_vel  = i;
            else if (!strcmp(fields[i],"heading"))       idx_head = i;
            else if (!strcmp(fields[i],"callsign"))      idx_callsign = i;
            else if (!strcmp(fields[i],"baroaltitude"))  idx_baro = i;
        }

        /* Fallback for legacy 8-col input (positional) */
        if (idx_time<0 && nf>=8){
            idx_time=0; idx_icao=1; idx_lat=2; idx_lon=3;
            idx_vel=4; idx_head=5; idx_callsign=6; idx_baro=7;
        }
    }

    while(fgets(line, MAX_LINE, fp)){
        rstrip_newline(line);

        if(*count >= *capacity){
            int newcap = (*capacity) * 2;
            if (newcap < 1) newcap = INIT_CAPACITY;
            Aircraft *tmp = (Aircraft*)realloc(*arr, (size_t)newcap * sizeof(Aircraft));
            if(!tmp){ perror("realloc"); fclose(fp); return -1; }
            *arr = tmp;
            *capacity = newcap;
        }

        char *fields[64];
        int nf = split_csv_simple(line, fields, 64);

        Aircraft ac; memset(&ac, 0, sizeof(ac));

        if (idx_time>=0 && idx_time<nf){
            strncpy(ac.time, fields[idx_time], 31);
            ac.time[31]='\0';
            rtrim_spaces(ac.time);
        }
        if (idx_icao>=0 && idx_icao<nf){
            strncpy(ac.icao24, fields[idx_icao], 15);
            ac.icao24[15]='\0';
            rtrim_spaces(ac.icao24);
        }
        if (idx_callsign>=0 && idx_callsign<nf){
            strncpy(ac.callsign, fields[idx_callsign], 15);
            ac.callsign[15]='\0';
            rtrim_spaces(ac.callsign);
        }
        if (idx_vel>=0    && idx_vel<nf)   ac.velocity     = parse_double_or0(fields[idx_vel]);
        if (idx_head>=0   && idx_head<nf)  ac.heading      = parse_double_or0(fields[idx_head]);
        if (idx_baro>=0   && idx_baro<nf)  ac.baroaltitude = parse_double_or0(fields[idx_baro]);

        double lat = 0.0, lon = 0.0;
        bool lat_ok = (idx_lat>=0 && idx_lat<nf) && parse_double_strict(fields[idx_lat], &lat);
        bool lon_ok = (idx_lon>=0 && idx_lon<nf) && parse_double_strict(fields[idx_lon], &lon);
        if (!lat_ok || !lon_ok || lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0){
            continue;  /* ignore this record */
        }
        ac.lat = lat; ac.lon = lon;

        (*arr)[(*count)++] = ac;
    }

    fclose(fp);
    return 0;
}

void print_top_from_idx(const Aircraft R[], const int *idx, int N, int X,
                        int phase, double ref_lat, double ref_lon, const char *label){

    if (N<=0 || X<=0) {
        printf("\n--- %s ---\n(no results)\n", label?label:"");
        return;
    }
    if (X > N) X = N;

    printf("\n--- %s ---\n", label?label:"Top");
    printf("  %-4s %-8s %-8s %9s %10s %8s %10s\n",
           "Rank","ICAO","Callsign","Lat(deg)","Lon(deg)","Alt","Dist(km)");
    printf("  %-4s %-8s %-10s %9s %10s %8s %10s\n",
           "----","--------","--------","---------","----------","--------","----------");

    for (int i=0;i<X;i++){
        int k = idx[i];
        double dist = 0.0;
        if      (phase==1) dist = R[k].metric;
        else if (phase==2) dist = R[k].metric;
        else               dist = full_haversine(ref_lat, ref_lon, R[k].lat, R[k].lon);

        printf("  %4d %-8s %-8s %9.4f %10.4f %8.0f %10.4f\n",
               i+1, dash_if_empty(R[k].icao24), dash_if_empty(R[k].callsign),
               R[k].lat, R[k].lon, R[k].baroaltitude, dist);
    }
}

void write_csv_from_idx(const char *filename, const Aircraft R[], const int *idx,
                        int X, int phase, double ref_lat, double ref_lon){
    if (X<=0) { printf("No rows to write.\n"); return; }
    FILE *fp = fopen(filename,"w");
    if(!fp){ perror("CSV write"); return; }
    fprintf(fp,"rank,time,icao24,callsign,lat,lon,baroaltitude,dist_km\n");
    for(int i=0;i<X;i++){
        int k = idx[i];
        double dist = 0.0;
        if      (phase==1) dist = R[k].metric;
        else if (phase==2) dist = distance_from_half(R[k].metric);
        else               dist = full_haversine(ref_lat, ref_lon, R[k].lat, R[k].lon);

        fprintf(fp,"%d,%s,%s,%s,%.6f,%.6f,%.1f,%.3f\n",
                i+1,R[k].time,R[k].icao24,R[k].callsign,
                R[k].lat,R[k].lon,R[k].baroaltitude,dist);
    }
    fclose(fp);
    printf("Results written to %s\n",filename);
}

void log_timing(const char *filename,const char *phase_name,
                int count,int cores,double runtime){
    FILE *fp=fopen(filename,"a");
    if(!fp){ perror("Timing log"); return; }
    fprintf(fp,"%s,%d,%d,%.6f\n",phase_name,count,cores,runtime);
    fclose(fp);
}

/* -------- heap for phase3 ---------- */
void heap_sift_up(Hit h[], int i){
    while(i>0){
        int p=(i-1)/2;
        if(h[p].dist_km>=h[i].dist_km) break;
        Hit t=h[p]; h[p]=h[i]; h[i]=t; i=p;
    }
}

void heap_sift_down(Hit h[], int n, int i){
    for(;;){
        int l=2*i+1,r=l+1,m=i;
        if(l<n&&h[l].dist_km>h[m].dist_km) m=l;
        if(r<n&&h[r].dist_km>h[m].dist_km) m=r;
        if(m==i) break;
        Hit t=h[i]; h[i]=h[m]; h[m]=t; i=m;
    }
}

void heap_push(Hit h[], int *n, Hit v){ h[(*n)++]=v; heap_sift_up(h,*n-1); }
Hit  heap_top(Hit h[]){ return h[0]; }
void heap_replace_top(Hit h[], int n, Hit v){ (void)n; h[0]=v; heap_sift_down(h,n,0); }

/* ---------- CLI ---------- */
Args parse_args(int argc, char *argv[]){
    Args a; memset(&a,0,sizeof(a));
    a.phase     = 0;
    a.output    = NULL; /* unused */
    a.ref_lat   = DEFAULT_LAT;
    a.ref_lon   = DEFAULT_LON;
    a.topX      = DEFAULT_X;
    a.data_file = DATA_FILE;

    for (int i=1; i<argc; ++i){
        if (starts_with(argv[i],"--phase="))                 a.phase = atoi(argv[i]+8);
        else if (!strcmp(argv[i],"--phase") && i+1<argc)     a.phase = atoi(argv[++i]);
        else if (starts_with(argv[i],"--data-file="))        a.data_file = argv[i] + 12;
        else if (!strcmp(argv[i],"--data-file") && i+1<argc) a.data_file = argv[++i];
    }

    /* Trailing [lat lon X] */
    if (argc>=4){
        char *e1=NULL,*e2=NULL,*e3=NULL;
        double tlat = strtod(argv[argc-3], &e1);
        double tlon = strtod(argv[argc-2], &e2);
        long   tX   = strtol(argv[argc-1], &e3, 10);
        if (e1 && *e1=='\0' && e2 && *e2=='\0' && e3 && *e3=='\0'){
            a.ref_lat = tlat; a.ref_lon = tlon; a.topX = (int)tX;
        }
    }
    return a;
}
