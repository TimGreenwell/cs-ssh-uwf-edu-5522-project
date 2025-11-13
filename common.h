#ifndef COMMON_H
#define COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef EARTH_RADIUS_KM
#define EARTH_RADIUS_KM 6371.0
#endif


#ifndef MAX_LINE
#define MAX_LINE 2048       /* Max size of line fetched from CSV */
#endif

#ifndef INIT_CAPACITY
#define INIT_CAPACITY 512   /* Initial Aircraft array size.. doubles as it fills */
#endif

/* CLI / app defaults (shared) */
#ifndef DEFAULT_LAT
#define DEFAULT_LAT 30.4733
#endif
#ifndef DEFAULT_LON
#define DEFAULT_LON -87.1866
#endif
#ifndef DEFAULT_X
#define DEFAULT_X 25
#endif
#ifndef DATA_FILE
#define DATA_FILE "input/input-data.csv"
#endif


/* ---------- types shared across units ---------- */
typedef struct {
    char time[32];
    char icao24[16];
    double lat, lon;
    double velocity;
    double heading;
    double baroaltitude;
    char callsign[16];
    double metric;   /* phase-dependent metric used for sorting */
    double dist_km;  /* final computed distance for printing/CSV */
} Aircraft;

typedef struct { double dist_km; int idx; } Hit;

typedef struct {
    int   phase;         /* 1,2,3 */
    char *output;        /* (ignored; output is phase-specific) */
    double ref_lat;      /* optional override */
    double ref_lon;
    int    topX;         /* optional override */
    const char *data_file; /* CLI overridable input path */
} Args;

/* ---------- tiny helpers ---------- */
static inline int starts_with(const char *s, const char *p) {
    return strncmp(s, p, strlen(p)) == 0;
}

/* ---------- utilities provided by common.c ---------- */
double wall_time(void);
int    ensure_dir(const char *path);

void rstrip_newline(char *s);
void rtrim_spaces(char *s);
int  split_csv_simple(char *line, char *fields[], int max_fields);
bool parse_double_strict(const char *s, double *out);
double parse_double_or0(const char *s);
const char* dash_if_empty(const char *s);

/* CSV I/O, printing, logging (these will call distance helpers provided in drivers) */
int  load_csv(const char *filename, Aircraft **arr, int *count, int *capacity);
void print_top_from_idx(const Aircraft R[], const int *idx, int N, int X,
                        int phase, double ref_lat, double ref_lon, const char *label);
void write_csv_from_idx(const char *filename, const Aircraft R[], const int *idx,
                        int X, int phase, double ref_lat, double ref_lon);
void log_timing(const char *filename,const char *phase_name,
                int count,int cores,double runtime);

/* compact max-heap used by phase 3 (provided in common.c) */
void heap_sift_up(Hit h[], int i);
void heap_sift_down(Hit h[], int n, int i);
void heap_push(Hit h[], int *n, Hit v);
Hit  heap_top(Hit h[]);
void heap_replace_top(Hit h[], int n, Hit v);

/* CLI parser shared by both drivers */
Args parse_args(int argc, char *argv[]);

/* NOTE: No declarations here for deg2rad/lon_diff/haversine/equirectangular
   and no declarations for sorting. Those live inside each driver. */

#endif /* COMMON_H */

