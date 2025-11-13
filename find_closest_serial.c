/***********************************************************************
 * Find Closest (Serial)
 *   - No OpenMP anywhere.
 *   - Sorts & haversine math live here.
 ***********************************************************************/

#define _POSIX_C_SOURCE 200809L
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <math.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>


/* ---------- app config ---------- */
#define EPSILON     0.05
#define LOG_FILE  "output/timings.csv"

/* ---------- math (haversine & helpers) ---------- */
static inline double deg2rad(double deg){ return deg * M_PI / 180.0; }

static inline double lon_diff(double lon1, double lon2){
    double dlon = lon1 - lon2;
    while(dlon > 180.0)  dlon -= 360.0;
    while(dlon < -180.0) dlon += 360.0;
    return dlon;
}

double full_haversine(double lat1,double lon1,double lat2,double lon2){
    double dlat  = deg2rad(lat2 - lat1);
    double dlon  = deg2rad(lon_diff(lon2, lon1));
    double rlat1 = deg2rad(lat1), rlat2 = deg2rad(lat2);
    double a = pow(sin(dlat/2.0),2.0) +
               cos(rlat1)*cos(rlat2)*pow(sin(dlon/2.0),2.0);
    return 2.0 * EARTH_RADIUS_KM * asin(sqrt(a));
}

double half_haversine(double lat1,double lon1,double lat2,double lon2){
    double dlat  = deg2rad(lat2 - lat1);
    double dlon  = deg2rad(lon_diff(lon2, lon1));
    double rlat1 = deg2rad(lat1), rlat2 = deg2rad(lat2);
    return pow(sin(dlat/2.0),2.0) +
           cos(rlat1)*cos(rlat2)*pow(sin(dlon/2.0),2.0);
}

double distance_from_half(double a){
    if (a < 0) a = 0;
    if (a > 1) a = 1;
    return 2.0 * EARTH_RADIUS_KM * asin(sqrt(a));
}

static inline double equirectangular(double lat1,double lon1,double lat2,double lon2){
    double dlon = lon_diff(lon2, lon1);
    double x = deg2rad(dlon) * cos(deg2rad((lat1 + lat2)/2.0));
    double y = deg2rad(lat2 - lat1);
    return EARTH_RADIUS_KM * sqrt(x*x + y*y);
}

/* ---------- sorts over index by R[].metric (serial) ---------- */
static inline int cmp_idx_metric(const Aircraft *R, int a, int b){
    if (R[a].metric < R[b].metric) return -1;
    if (R[a].metric > R[b].metric) return  1;
    return (a<b)?-1:(a>b);
}

static void odd_even_sort_idx(const Aircraft *R, int *idx, int n){
    int sorted = 0;
    while (!sorted){
        sorted = 1;
        for (int i=1; i+1<n; i+=2){
            if (cmp_idx_metric(R, idx[i], idx[i+1]) > 0){
                int t=idx[i]; idx[i]=idx[i+1]; idx[i+1]=t; sorted = 0;
            }
        }
        for (int i=0; i+1<n; i+=2){
            if (cmp_idx_metric(R, idx[i], idx[i+1]) > 0){
                int t=idx[i]; idx[i]=idx[i+1]; idx[i+1]=t; sorted = 0;
            }
        }
    }
}

static void merge_idx(const Aircraft *R, int *idx, int *buf, int l, int m, int r){
    int i=l, j=m, k=l;
    while (i<m && j<r){
        if (cmp_idx_metric(R, idx[i], idx[j]) <= 0) buf[k++] = idx[i++];
        else                                         buf[k++] = idx[j++];
    }
    while (i<m) buf[k++] = idx[i++];
    while (j<r) buf[k++] = idx[j++];
    for (int x=l; x<r; ++x) idx[x] = buf[x];
}
static void mergesort_idx_rec(const Aircraft *R, int *idx, int *buf, int l, int r){
    if (r - l <= 32){
        for (int i=l+1; i<r; ++i){
            int key = idx[i], j = i-1;
            while (j>=l && cmp_idx_metric(R, idx[j], key) > 0){
                idx[j+1]=idx[j]; --j;
            }
            idx[j+1]=key;
        }
        return;
    }
    int m = l + ((r-l)>>1);
    mergesort_idx_rec(R, idx, buf, l, m);
    mergesort_idx_rec(R, idx, buf, m, r);
    if (cmp_idx_metric(R, idx[m-1], idx[m]) <= 0) return;
    merge_idx(R, idx, buf, l, m, r);
}
static void merge_sort_idx(const Aircraft *R, int *idx, int n){
    int *buf = (int*)malloc(sizeof(int)*n);
    if (!buf){ fprintf(stderr,"OOM merge buf\n"); exit(1); }
    mergesort_idx_rec(R, idx, buf, 0, n);
    free(buf);
}


/* ---------- reporting ---------- */
static bool same_topX2(const int *A, const int *B, int X){
    for (int i=0;i<X;i++) if (A[i]!=B[i]) return false;
    return true;
}

static void run_two_sorts_fullarray_and_report(Aircraft *R, int count, int X,
                                               int phase, double ref_lat, double ref_lon)
{
    if (count <= 0 || X <= 0){ puts("\n(no results)"); return; }
    int limit = (X < count) ? X : count;

    int *base = (int*)malloc(sizeof(int)*count);
    int *idx_odd   = (int*)malloc(sizeof(int)*count);
    int *idx_merge = (int*)malloc(sizeof(int)*count);
    if(!base||!idx_odd||!idx_merge){ fprintf(stderr,"OOM idx\n"); exit(1); }

    for (int i=0;i<count;i++) base[i]=i;
    memcpy(idx_odd, base, sizeof(int)*count);
    memcpy(idx_merge, base, sizeof(int)*count);

    double t0, t1;
    t0 = wall_time(); odd_even_sort_idx(R, idx_odd, count);  t1 = wall_time();
    double ms_odd = (t1 - t0) * 1000.0;

    t0 = wall_time(); merge_sort_idx(R, idx_merge, count);   t1 = wall_time();
    double ms_merge = (t1 - t0) * 1000.0;

    printf("[Time] Even–Odd Transposition Sort (full array): %.3f ms\n", ms_odd);
    printf("[Time] Merge Sort (full array)                : %.3f ms\n", ms_merge);

    if (same_topX2(idx_odd, idx_merge, limit)){
        print_top_from_idx(R, idx_merge, count, X, phase, ref_lat, ref_lon,
                           "Top-X (both sorts agree)");
    } else {
        print_top_from_idx(R, idx_odd,   count, X, phase, ref_lat, ref_lon, "Top-X [Even–Odd]");
        print_top_from_idx(R, idx_merge, count, X, phase, ref_lat, ref_lon, "Top-X [Merge]");
    }

    const char *auto_name =
        (phase==1) ? "output/phase1-output.csv" :
        (phase==2) ? "output/phase2-output.csv" : "output/phaseX-output.csv";
    write_csv_from_idx(auto_name, R, idx_merge, limit, phase, ref_lat, ref_lon);

    free(idx_merge); free(idx_odd); free(base);
}

/* ---------- phases ---------- */
static void phase1(Aircraft *aircrafts,int count,int X,double ref_lat,double ref_lon){
    double t0=wall_time();
    for(int i=0;i<count;i++){
        aircrafts[i].metric = full_haversine(ref_lat,ref_lon,
                                             aircrafts[i].lat,aircrafts[i].lon);
    }
    double t1=wall_time();
    printf("Phase 1 metric runtime: %.3f s\n",t1-t0);
    log_timing(LOG_FILE,"Phase1_FullHaversine",count,1,t1-t0);
    run_two_sorts_fullarray_and_report(aircrafts, count, X, 1, ref_lat, ref_lon);
}

static void phase2(Aircraft *aircrafts,int count,int X,double ref_lat,double ref_lon){
    double t0=wall_time();
    for(int i=0;i<count;i++){
        aircrafts[i].metric = half_haversine(ref_lat,ref_lon,
                                             aircrafts[i].lat,aircrafts[i].lon);
    }
    double t1=wall_time();
    printf("Phase 2 metric runtime: %.3f s\n",t1-t0);
    log_timing(LOG_FILE,"Phase2_HalfHaversine",count,1,t1-t0);
    run_two_sorts_fullarray_and_report(aircrafts, count, X, 2, ref_lat, ref_lon);
}

static int cmp_hit_dist(const void *p1,const void *p2){
    double a=((const Hit*)p1)->dist_km, b=((const Hit*)p2)->dist_km;
    return (a>b) - (a<b);
}

static void phase3(Aircraft *arr,int count,int X,double ref_lat,double ref_lon){
    double coslat = cos(deg2rad(ref_lat));
    if (fabs(coslat) < 1e-6) coslat = 1e-6;
    int use_lon = fabs(ref_lat) < 80.0;

    double t0 = wall_time();
    if (X <= 0 || count <= 0){ puts("No candidates after streaming."); return; }

    Hit *heap = (Hit*)malloc((size_t)X * sizeof(Hit));
    int k = 0;
    double cutoff_km = INFINITY;
    double lat_thresh_deg = INFINITY;
    double lon_thresh_deg = use_lon ? INFINITY : 360.0;
    int bbox_survivors = 0;

    for (int i=0; i<count; ++i){
        if (k == X){
            double dlat = fabs(arr[i].lat - ref_lat);
            if (dlat > lat_thresh_deg) continue;
            if (use_lon){
                double dlon = fabs(lon_diff(arr[i].lon, ref_lon));
                if (dlon > lon_thresh_deg) continue;
            }
        }

        double d_eq = equirectangular(ref_lat, ref_lon, arr[i].lat, arr[i].lon);
        ++bbox_survivors;

        if (k < X){
            heap_push(heap, &k, (Hit){ d_eq, i });
            if (k == X){
                cutoff_km      = heap_top(heap).dist_km * (1.0 + EPSILON);
                lat_thresh_deg = cutoff_km / 111.0;
                lon_thresh_deg = use_lon ? cutoff_km / (111.0 * coslat) : 360.0;
            }
        } else {
            double worst = heap_top(heap).dist_km;
            if (d_eq <= worst * (1.0 + EPSILON)){
                heap_replace_top(heap, k, (Hit){ d_eq, i });
                cutoff_km      = heap_top(heap).dist_km * (1.0 + EPSILON);
                lat_thresh_deg = cutoff_km / 111.0;
                lon_thresh_deg = use_lon ? cutoff_km / (111.0 * coslat) : 360.0;
            }
        }
    }

    if (k == 0){
        puts("No candidates after streaming.");
        free(heap);
        return;
    }

    Hit *refined = (Hit*)malloc((size_t)k * sizeof(Hit));
    for (int j=0; j<k; ++j){
        int idx = heap[j].idx;
        refined[j].idx     = idx;
        refined[j].dist_km = full_haversine(ref_lat, ref_lon, arr[idx].lat, arr[idx].lon);
    }
    qsort(refined, (size_t)k, sizeof(Hit), cmp_hit_dist);

    int limit = (X < k) ? X : k;
    int *idx_refined = (int*)malloc(sizeof(int)*k);
    for (int i=0; i<k; ++i) idx_refined[i] = refined[i].idx;

    print_top_from_idx(arr, idx_refined, k, limit, 3, ref_lat, ref_lon,
                       "Phase 3 (Adaptive Pruning)");
    write_csv_from_idx("output/phase3-output.csv", arr, idx_refined, limit, 3, ref_lat, ref_lon);

    free(idx_refined);
    free(refined);
    free(heap);

    double t1 = wall_time();
    printf("Bounding-box survivors: %d of %d\n", bbox_survivors, count);
    printf("Post-equirectangular survivors: %d\n", k);
    printf("Phase 3 runtime: %.3f s\n", t1 - t0);
    log_timing(LOG_FILE,"Phase3_PruneRefine",count,1,t1-t0);
}

/* ---------- main ---------- */
int main(int argc,char *argv[]){
    Args args = parse_args(argc, argv);

    if (args.phase < 1 || args.phase > 3){
        fprintf(stderr,
            "Usage: %s --phase {1|2|3} [lat lon X] [--data-file <path>]\n", argv[0]);
        fprintf(stderr,
            "Example: %s --phase=3 30.4733 -87.1866 25 --data-file input/myfile.csv\n", argv[0]);
        return 1;
    }

    const char *auto_name =
        (args.phase==1) ? "output/phase1-output.csv" :
        (args.phase==2) ? "output/phase2-output.csv" :
                          "output/phase3-output.csv";

    if (ensure_dir("output") != 0){
        perror("ensure_dir(\"output\")");
    }

    printf("Phase: %d\n", args.phase);
    printf("Output: %s\n", auto_name);
    printf("Ref: (%.4f, %.4f), TopX=%d\n", args.ref_lat, args.ref_lon, args.topX);
    printf("Data file: %s\n", args.data_file);

    int capacity = INIT_CAPACITY;
    int count = 0;
    Aircraft *aircrafts = (Aircraft*)malloc((size_t)capacity*sizeof(Aircraft));
    if (!aircrafts){ perror("malloc"); return 1; }

    if (load_csv(args.data_file, &aircrafts, &count, &capacity) != 0){
        fprintf(stderr,"Failed to load %s\n", args.data_file);
    }
    printf("Loaded %d records\n", count);
    if (count==0){ free(aircrafts); return 0; }

    if      (args.phase==1) phase1(aircrafts,count,args.topX,args.ref_lat,args.ref_lon);
    else if (args.phase==2) phase2(aircrafts,count,args.topX,args.ref_lat,args.ref_lon);
    else                    phase3(aircrafts,count,args.topX,args.ref_lat,args.ref_lon);

    free(aircrafts);
    return 0;
}

