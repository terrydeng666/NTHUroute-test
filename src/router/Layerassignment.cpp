#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)
// #define FOLLOW_PREFER
//  #define VIA_DENSITY	// new experiment 2007/09/27
// #define REDUCE_VIA_DENSITY	// new experiment 2007/09/29
#define CHECK_PREFER
#define MAX_OVERFLOW_CONSTRAINT
#define FROM2D
#define ALLOUTPUT

#define PIN_LEF
// #define PATCHING
// #define DAC2023
// #define VERIFY_PIN_LEF
// #define PRINT_DP_PIN
// #define VERIFY_PIN_1
#include "Construct_2d_tree.h"
#include "misc/geometry.h"
#include <algorithm>
#include <queue>
#include <set>
#include <string.h>
#include <time.h>
#include <tuple>
using namespace std;
using namespace Jm;

#ifdef CHECK_PREFER
char prefer_direction[6][2] = {0};
#endif
int tar = -2;
char follow_prefer_direction;
// enum {GREEDY, SHORT_PATH};
int l_option;
int max_xx, max_yy, max_zz, overflow_max, *prefer_idx;
Coordinate_3d ***coord_3d_map;
int i_router, i_test_case, i_order, i_method;
const char temp_buf[1000] = "1000";
bool check_net = false;
bool is_metal5 = false;
extern std::string TESTCASE_NAME;
extern std::chrono::high_resolution_clock::time_point io_start;
extern std::chrono::high_resolution_clock::time_point io_end;

extern std::vector<int> layerDirections;

long long total_via = 0;

double DP_TIME = 0;

int CNT = 1;

struct
{
    int idx;
    int val;
} ans;

typedef struct
{
    int val;
    Coordinate_3d *pi;
} DP_NODE;
DP_NODE ***dp_map;

typedef struct
{
    int id;
    int val;
} NET_NODE;
NET_NODE *net_order;

typedef struct
{
    int id;
    int times;
    int val;
    int vo_times;
    double average;
    int bends;
} AVERAGE_NODE;
AVERAGE_NODE *average_order;

typedef struct
{
    int xy;
    int z;
    double val;
} NET_INFO_NODE;
NET_INFO_NODE *net_info;

void update_ans(int tar, int val)
{
    ans.idx = tar;
    ans.val = val;
}

typedef struct
{
    Two_pin_list_2d two_pin_net_list;
} MULTIPIN_NET_NODE;
MULTIPIN_NET_NODE *multi_pin_net;

typedef struct
{
    int val;
    int edge[4];
    int pin_layer; // may be a problem, because there are pins in same (x, y), but not the same layer
                   // ????????
} PATH_NODE;
PATH_NODE **path_map;

typedef struct
{
    int val;
    int via_cost;
    int via_overflow;
    int pi_z;
} KLAT_NODE;
KLAT_NODE ***klat_map;

typedef struct
{
    int *edge[4];
} OVERFLOW_NODE;
OVERFLOW_NODE **overflow_map;

typedef struct
{
    int pi;
    int sx, sy, bx, by;
    int num;
} UNION_NODE;
UNION_NODE *group_set;

typedef struct
{
    int xy;
    int z;
} LENGTH_NODE;
LENGTH_NODE length_count[1000];

struct VIADENSITY_NODE
{
    int cur = 0;
    int max = 0;
};

std::vector<std::vector<std::vector<VIADENSITY_NODE>>> viadensity_map;

typedef struct
{
    set<int> used_net;
} PATH_EDGE_3D;
typedef PATH_EDGE_3D *PATH_EDGE_3D_PTR;

typedef struct
{
    PATH_EDGE_3D_PTR edge_list[6];
} PATH_VERTEX_3D;
PATH_VERTEX_3D ***path_map_3d;

const int plane_dir[4][2] = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};                                   // F B L R
const int cube_dir[6][3] = {{0, 1, 0}, {0, -1, 0}, {-1, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, -1}}; // F B L R U D
int global_net_id, global_x, global_y, global_max_layer, global_pin_num, global_xy_reduce = 0, global_BFS_xy = 0;
long long global_pin_cost = 0;
int min_DP_val, min_DP_idx[4], max_DP_k, min_DP_k, min_DP_via_cost;
int total_pin_number = 0;
int ***BFS_color_map;
int temp_global_pin_cost, temp_global_xy_cost, after_xy_cost;
// int is_used_for_BFS[1300][1300]; // modified
int global_increase_vo;
vector<int> lost_pin_net;
vector<vector<pair<Coordinate_3d, Coordinate_3d>>> wire_result; // wire
vector<vector<pair<Coordinate_3d, Coordinate_3d>>> via_result;  // via
// std::ofstream fout_for_txt("overflow.txt");
std::ofstream fout_tmp;
int lost_pin = 0;

inline std::string viaToGuide(int x, int y, int layer1, int layer2)
{
    auto pointToGcell = [&](int x, int y, int xStep, int yStep)
    {
        std::vector<utils::PointT<int>> coord(2);
        coord[0].x = max(0, x * xStep - 1);
        coord[0].y = max(0, y * yStep - 1);
        coord[1].x = max(0, (x + 1) * xStep - 1);
        coord[1].y = max(0, (y + 1) * yStep - 1);
        return coord;
    };

    std::string str = "";
    int layer_1 = std::min(layer1, layer2);
    int layer_2 = std::max(layer1, layer2);

    /*(2024/01/21) MF edited
    for (int l = layer_1; l <= layer_2; ++l) {
        str += std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(x+1) + " " + std::to_string(y+1) + "
    metal" + std::to_string(l) + "\n";
    }
    */
    for (int l = layer_1; l < layer_2; ++l)
    {
        str += std::to_string(x * 4200 + 2100) + " " + std::to_string(y * 4200 + 2100) + " metal" + std::to_string(l + 1) + " " + std::to_string(x * 4200 + 2100) + " " // 1/20 via+1
               + std::to_string(y * 4200 + 2100) + " metal" + std::to_string(l + 2) + "\n";
    }
    // std::cout << str << endl;
    return str;

    auto coord1 = pointToGcell(x, y, grDatabase.getXStep(), grDatabase.getYStep());
    int xmax = std::min(int(database.dieRegion.x.high), coord1[1].x);
    int ymax = std::min(int(database.dieRegion.y.high), coord1[1].y);
    for (int i = layer_1; i < layer_2 + 1; i++)
    {
        str = str + std::to_string(coord1[0].x) + " " + std::to_string(coord1[0].y) + " " + std::to_string(coord1[1].x) + " " + std::to_string(coord1[1].y) + " Metal" + std::to_string(i) + "\n";
    }
    return str;
}

inline std::string rectToGuide(int x1, int y1, int x2, int y2, int layer)
{
    auto pointToGcell = [&](int x, int y, int xStep, int yStep)
    {
        std::vector<utils::PointT<int>> coord(2);
        coord[0].x = max(0, x * xStep - 1);
        coord[0].y = max(0, y * yStep - 1);
        coord[1].x = max(0, (x + 1) * xStep - 1);
        coord[1].y = max(0, (y + 1) * yStep - 1);
        return coord;
    };

    std::string str;
    /* (2024/01/21) MF edited
    str = std::to_string(min(x1, x2)) + " " + std::to_string(min(y1, y2)) + " ";
    str += std::to_string(max(x1, x2) + 1) + " " + std::to_string(max(y1, y2) + 1) + " ";
    str += "metal" + std::to_string(layer) + "\n";
    */
    // 2025/1/20 layer -1  => layer
    str = std::to_string(min(x1, x2) * 4200 + 2100) + " " + std::to_string(min(y1, y2) * 4200 + 2100) + " metal" + std::to_string(layer + 1) + " " + std::to_string(max(x1, x2) * 4200 + 2100) + " " + std::to_string(max(y1, y2) * 4200 + 2100) + " metal" + std::to_string(layer + 1) + "\n";
    return str;

    auto coord1 = pointToGcell(x1, y1, grDatabase.getXStep(), grDatabase.getYStep());
    auto coord2 = pointToGcell(x2, y2, grDatabase.getXStep(), grDatabase.getYStep());
    int xmax = INT_MIN, xmin = INT_MAX;
    int ymax = INT_MIN, ymin = INT_MAX;
    coord1.insert(coord1.end(), coord2.begin(), coord2.end());
    for (auto &point : coord1)
    {
        if (point.x > xmax)
            xmax = point.x;
        if (point.x < xmin)
            xmin = point.x;
        if (point.y > ymax)
            ymax = point.y;
        if (point.y < ymin)
            ymin = point.y;
    }

    str = str + std::to_string(xmin) + " ";
    str = str + std::to_string(ymin) + " ";
    if (xmax > database.dieRegion.x.high)
        str = str + std::to_string(database.dieRegion.x.high) + " ";
    else
        str = str + std::to_string(xmax) + " ";
    if (ymax > database.dieRegion.y.high)
        str = str + std::to_string(database.dieRegion.y.high) + " ";
    else
        str = str + std::to_string(ymax) + " ";
    str = str + "Metal" + std::to_string(layer) + "\n";
    return str;
}

void print_max_overflow(void)
{
    auto sign = [](double x)
    {
        const double EPS = 1e-8;
        if (abs(x) < EPS)
            return 0;
        if (x + EPS < 0)
            return -1;
        return 1;
    };

    int i, j, k, lines = 0; //, ii, jj, dir;
    int max = 0;
    int sum = 0, temp_sum;
    // int overflow_val;
    // int x, y;
    vector<int> sum_overflow;

    for (int i = 0; i < max_zz; i++)
    {
        sum_overflow.push_back(0);
    }

    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
        {
            temp_sum = 0;
            for (k = 0; k < max_zz; ++k)
            {
                temp_sum += cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap;
                // if (/*k == 0 || */database.getLayerDir(k) == X) {
                // 	assert(cur_map_3d[i][j][k].edge_list[LEFT]->max_cap == 0);
                // 	assert(cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap == 0);
                // }
                if (cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap > cur_map_3d[i][j][k].edge_list[LEFT]->max_cap) // overflow
                {
                    // printf("%d %d %d, cur_cap: %d, max_cap: %d\n", i, j, k,
                    // cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap, cur_map_3d[i][j][k].edge_list[LEFT]->max_cap);

                    // if((cur_map_3d[i][j][k].edge_list[LEFT]->max_cap % 2) != 0) // modified
                    // 	cur_map_3d[i][j][k].edge_list[LEFT]->max_cap -= 1; // modified
                    if (cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap - cur_map_3d[i][j][k].edge_list[LEFT]->max_cap > max)
                    {
                        max =
                            cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap - cur_map_3d[i][j][k].edge_list[LEFT]->max_cap;
                    }
                    sum +=
                        (cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap - cur_map_3d[i][j][k].edge_list[LEFT]->max_cap);
                    sum_overflow[k] +=
                        (cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap - cur_map_3d[i][j][k].edge_list[LEFT]->max_cap);
                    lines++;
                }
            }
        }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j)
        {
            temp_sum = 0;
            for (k = 0; k < max_zz; ++k)
            {
                temp_sum += cur_map_3d[i][j][k].edge_list[BACK]->cur_cap;
                // if (/*k == 0 || */database.getLayerDir(k) == Y) {
                // 	assert(cur_map_3d[i][j][k].edge_list[BACK]->max_cap == 0);
                // 	assert(cur_map_3d[i][j][k].edge_list[BACK]->cur_cap == 0);
                // }
                if (cur_map_3d[i][j][k].edge_list[BACK]->cur_cap > cur_map_3d[i][j][k].edge_list[BACK]->max_cap) // overflow
                {
                    if (cur_map_3d[i][j][k].edge_list[BACK]->cur_cap - cur_map_3d[i][j][k].edge_list[BACK]->max_cap > max)
                    {
                        max =
                            cur_map_3d[i][j][k].edge_list[BACK]->cur_cap - cur_map_3d[i][j][k].edge_list[BACK]->max_cap;
                    }
                    sum +=
                        (cur_map_3d[i][j][k].edge_list[BACK]->cur_cap - cur_map_3d[i][j][k].edge_list[BACK]->max_cap);
                    sum_overflow[k] +=
                        (cur_map_3d[i][j][k].edge_list[BACK]->cur_cap - cur_map_3d[i][j][k].edge_list[BACK]->max_cap);
                    lines++;
                }
            }
        }

    for (int cnt_layer = 0; cnt_layer < max_zz; cnt_layer++)
    {
        std::cout << "layer: " << cnt_layer << "  3D # of overflow = " << sum_overflow[cnt_layer] << std::endl;
    }
    printf("3D # of overflow = %d\n", sum);
    printf("3D max overflow = %d\n", max);
    printf("3D overflow edge number = %d\n", lines);
}

void find_overflow_max(void)
{
    int i, j;

    overflow_max = 0;
    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            if (congestionMap2d->edge(i, j, DIR_WEST).overUsage() > overflow_max)
                overflow_max = congestionMap2d->edge(i, j, DIR_WEST).overUsage();
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j)
            if (congestionMap2d->edge(i, j, DIR_SOUTH).overUsage() > overflow_max)
                overflow_max = congestionMap2d->edge(i, j, DIR_SOUTH).overUsage();
    printf("2D maximum overflow = %d\n", overflow_max);
#ifdef MAX_OVERFLOW_CONSTRAINT
#ifdef FOLLOW_PREFER
    if (follow_prefer_direction == 1)
    {
        if (overflow_max % (max_zz / 2))
            overflow_max = ((overflow_max / (max_zz / 2)) << 1) + 2;
        else
            overflow_max = ((overflow_max / (max_zz / 2)) << 1);
    }
    else
    {
        if (overflow_max % max_zz)
            overflow_max = ((overflow_max / max_zz) << 1) + 2;
        else
            overflow_max = ((overflow_max / max_zz) << 1);
    }
#else
    // modified
    overflow_max = static_cast<int>(std::ceil((double)overflow_max / ((max_zz - 1) / 2)));
    // overflow_max = static_cast<int>(std::ceil((double)overflow_max/(max_zz/2)));
    // overflow_max << 1;
    // if (overflow_max % max_zz)
    // 	overflow_max = ((overflow_max / max_zz) << 1) + 1; //modified
    // else
    // 	overflow_max = ((overflow_max / max_zz) << 1);
#endif
#else
    overflow_max <<= 1;
#endif
    printf("overflow max = %d\n", overflow_max);
}

void initial_3D_coordinate_map(void)
{
    int i, j, k;

    coord_3d_map = (Coordinate_3d ***)malloc(sizeof(Coordinate_3d **) * max_xx);
    for (i = 0; i < max_xx; ++i)
    {
        coord_3d_map[i] = (Coordinate_3d **)malloc(sizeof(Coordinate_3d *) * max_yy);
        for (j = 0; j < max_yy; ++j)
            coord_3d_map[i][j] = (Coordinate_3d *)malloc(sizeof(Coordinate_3d) * max_zz);
    }
    for (i = 0; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            for (k = 0; k < max_zz; ++k)
            {
                coord_3d_map[i][j][k].x = i;
                coord_3d_map[i][j][k].y = j;
                coord_3d_map[i][j][k].z = k;
            }
}

void malloc_path_map(void)
{
    int i, j, k;
    path_map = (PATH_NODE **)malloc(sizeof(PATH_NODE *) * max_xx);
    for (i = 0; i < max_xx; ++i)
    {
        path_map[i] = (PATH_NODE *)malloc(sizeof(PATH_NODE) * max_yy);
    }
    // initial path_map
    for (i = 0; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
        {
            path_map[i][j].val = 0; // non-visited
            path_map[i][j].pin_layer = -1;
            for (k = 0; k < 4; ++k)
                path_map[i][j].edge[k] = 0;
        }
}

void malloc_klat_map(void)
{
    int i, j;

    klat_map = (KLAT_NODE ***)malloc(sizeof(KLAT_NODE **) * max_xx);
    for (i = 0; i < max_xx; ++i)
    {
        klat_map[i] = (KLAT_NODE **)malloc(sizeof(KLAT_NODE *) * max_yy);
        for (j = 0; j < max_yy; ++j)
            klat_map[i][j] = (KLAT_NODE *)malloc(sizeof(KLAT_NODE) * max_zz);
    }
}

void malloc_overflow_map(void)
{
    int i, j;
    int *temp;

    overflow_map = (OVERFLOW_NODE **)malloc(sizeof(OVERFLOW_NODE *) * max_xx);
    for (i = 0; i < max_xx; ++i)
        overflow_map[i] = (OVERFLOW_NODE *)malloc(sizeof(OVERFLOW_NODE) * max_yy);
    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
        {
            temp = (int *)malloc(sizeof(int));
            overflow_map[i][j].edge[LEFT] = temp;
            overflow_map[i - 1][j].edge[RIGHT] = temp;
        }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j)
        {
            temp = (int *)malloc(sizeof(int));
            overflow_map[i][j].edge[BACK] = temp;
            overflow_map[i][j - 1].edge[FRONT] = temp;
        }
}

void initial_overflow_map(void)
{
    int i, j;
    int max_of_quota = INT_MIN;
    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
        {
            *(overflow_map[i][j].edge[LEFT]) = (congestionMap2d->edge(i, j, DIR_WEST).overUsage()); // modified
            max_of_quota = max(max_of_quota, *(overflow_map[i][j].edge[LEFT]));
        }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j)
        {
            *(overflow_map[i][j].edge[BACK]) = (congestionMap2d->edge(i, j, DIR_SOUTH).overUsage()); // modified
            // if (i == 94 && j == 112) {
            // 	printf("%d %d\n", *(overflow_map[i][j].edge[BACK]), (congestionMap2d->edge(i, j,
            // DIR_SOUTH).overUsage())); 	printf("%lf %lf\n", congestionMap2d->edge(i, j, DIR_SOUTH).cur_cap,
            // congestionMap2d->edge(i, j, DIR_SOUTH).max_cap); 	printf("%lf\n", congestionMap2d->edge(i, j,
            // DIR_SOUTH).cur_cap-congestionMap2d->edge(i, j, DIR_SOUTH).max_cap);

            // 	assert(*(overflow_map[i][j].edge[BACK]) == *(overflow_map[i][j-1].edge[FRONT]));
            // 	assert(round(congestionMap2d->edge(i, j, DIR_SOUTH).cur_cap-congestionMap2d->edge(i, j,
            // DIR_SOUTH).max_cap) == 1);
            // }
            max_of_quota = max(max_of_quota, *(overflow_map[i][j].edge[BACK]));
        }
    printf("max_of_quota: %d\n", max_of_quota);
    // printf("(94, 111) of_map %d", *(overflow_map[94][111].edge[FRONT]));
    // printf("(94, 111) congestionMap2d cur_cap: %lf, max_cap: %lf\n", congestionMap2d->edge(94, 111,
    // DIR_NORTH).cur_cap, congestionMap2d->edge(94, 111, DIR_NORTH).max_cap);
}

void malloc_viadensity_map(void)
{

    viadensity_map.assign(max_xx, std::vector<std::vector<VIADENSITY_NODE>>(max_yy, std::vector<VIADENSITY_NODE>(max_zz)));
}

void malloc_space(void)
{
    malloc_path_map();
    malloc_klat_map();
    malloc_overflow_map();
    malloc_viadensity_map();
}

void free_path_map(void)
{
    int i;

    for (i = 0; i < max_xx; ++i)
        free(path_map[i]);
    free(path_map);
}

void free_klat_map(void)
{
    int i, j;

    for (i = 0; i < max_xx; ++i)
    {
        for (j = 0; j < max_yy; ++j)
            free(klat_map[i][j]);
        free(klat_map[i]);
    }
    free(klat_map);
}

void free_overflow_map(void)
{
    int i, j;

    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            free(overflow_map[i][j].edge[LEFT]);
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j)
            free(overflow_map[i][j].edge[BACK]);
    for (i = 0; i < max_xx; ++i)
        free(overflow_map[i]);
    free(overflow_map);
}

void free_viadensity_map(void)
{
}

void free_malloc_space(void)
{
    free_path_map();
    free_klat_map();
    free_overflow_map();
}

void update_cur_map_for_klat_xy(int cur_idx, Coordinate_2d *start, Coordinate_2d *end)
{
    int dir_idx;
    assert(cur_idx > 0);
    if (start->x != end->x && start->y == end->y)
    {
        int dir_idx = ((end->x > start->x) ? LEFT : RIGHT);
        int x_dir = ((end->x > start->x) ? 1 : -1);
        // std::cout << "dir idx " << dir_idx << " " << x_dir << endl;

        for (int i = start->x + x_dir; i != end->x + x_dir; i += x_dir)
        {
            // std::cout << i << endl;
            cur_map_3d[i][start->y][cur_idx].edge_list[dir_idx]->cur_cap++;
            // fout_for_txt << i << "\t" << start->y << "\t" << cur_idx << "\t" << ((dir_idx == LEFT) ? "LEFT" : "RIGHT") << endl;

            if (cur_map_3d[i][start->y][cur_idx].edge_list[dir_idx]->cur_cap > cur_map_3d[i][start->y][cur_idx].edge_list[dir_idx]->max_cap) // need check
            {
                // printf("%d %d %d, cur_cap: %lf, max_cap: %lf\n", end->x, end->y, cur_idx,
                // cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->cur_cap,
                // cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->max_cap);
                // if (global_net_id == 22933)
                //    std::cout << i << " " << start->y << " " << ((dir_idx == LEFT) ? "LEFT" : "RIGHT") << " " << *(overflow_map[i][start->y].edge[dir_idx]) << endl;
                *(overflow_map[i][start->y].edge[dir_idx]) -= 1; // modified
                assert(*(overflow_map[i][start->y].edge[dir_idx]) >= 0);
            }
        }
    }
    else if (start->y != end->y && start->x == end->x)
    {
        int dir_idx = ((end->y > start->y) ? BACK : FRONT);
        int y_dir = ((end->y > start->y) ? 1 : -1);

        for (int i = start->y + y_dir; i != end->y + y_dir; i += y_dir)
        {
            cur_map_3d[start->x][i][cur_idx].edge_list[dir_idx]->cur_cap++; // modified
            // fout_for_txt << start->x << "\t" << i << "\t" << cur_idx << "\t" << ((dir_idx == BACK) ? "BACK" : "FRONT") << endl;
            if (cur_map_3d[start->x][i][cur_idx].edge_list[dir_idx]->cur_cap > cur_map_3d[start->x][i][cur_idx].edge_list[dir_idx]->max_cap) // need check
            {
                // if (global_net_id == 22933)
                //     std::cout << start->x << " " << i << " " << ((dir_idx == BACK) ? "BACK" : "FRONT") << " " << *(overflow_map[start->x][i].edge[dir_idx]) << endl;

                *(overflow_map[start->x][i].edge[dir_idx]) -= 1; // modified
                assert(*(overflow_map[start->x][i].edge[dir_idx]) >= 0);
            }
        }
    }
    else if (start->x != end->x && start->y != end->y)
    {
        printf("%d %d %d %d\n", start->x, start->y, end->x, end->y);
        puts("ERROR!!!");
    }
}

void update_cur_map_for_klat_z(int pre_idx, int cur_idx, Coordinate_2d *start)
{

    int i, j, k, z_dir;

    z_dir = ((cur_idx > pre_idx) ? 1 : -1);
    i = start->x;
    j = start->y;
    for (k = pre_idx; k != cur_idx; k += z_dir)
    {
        // if (global_net_id == 51187)
        // std::cout << "for " << k << endl;
        if (z_dir == 1)
            ++viadensity_map[i][j][k].cur;
        else
            ++viadensity_map[i][j][k - 1].cur;
    }
}

void update_path_for_klat(Coordinate_2d *start, int start_pin_layer)
{
    int i, j, z_min, z_max, pin_num = 0, temp_z;
    queue<Coordinate_3d *> q; // do not need pointer
    Coordinate_3d *cur;
    auto end = std::unique_ptr<Jm::Coordinate_3d>(new Jm::Coordinate_3d());
    auto next = std::unique_ptr<Jm::Coordinate_3d>(new Jm::Coordinate_3d());
    vector<pair<Coordinate_3d, Coordinate_3d>> two_pin_nets; // wire
    vector<pair<Coordinate_3d, Coordinate_3d>> via;          // via
    // fout_for_txt << rr_map->get_net_name(global_net_id) << " " << global_net_id << endl;

    // BFS
    q.push(&coord_3d_map[start->x][start->y][start_pin_layer]); // enqueue

    while (!q.empty())
    {
        cur = (Coordinate_3d *)q.front();

        if (path_map[cur->x][cur->y].val == 2) // pin
        {

            pin_num++;

            if (cur->z > path_map[cur->x][cur->y].pin_layer)
            {
                z_min = path_map[cur->x][cur->y].pin_layer;
                z_max = cur->z;
            }
            else
            {
                z_min = cur->z;
                z_max = path_map[cur->x][cur->y].pin_layer;
            }
        }
        else // node
        {
            z_min = cur->z;
            z_max = cur->z;
        }

        for (i = 0; i < 4; ++i)
        {
            // std::cout << "dir " << i << " " << path_map[cur->x][cur->y].edge[i] << endl;
            if (path_map[cur->x][cur->y].edge[i] == 1) // check leagal
            {
                // legal end point
                end->x = cur->x + plane_dir[i][0];
                end->y = cur->y + plane_dir[i][1];
                temp_z = cur->z;

                // update z range
                z_max = max(z_max, klat_map[end->x][end->y][cur->z].pi_z);
                z_min = min(z_min, klat_map[end->x][end->y][cur->z].pi_z);

                // move to correspond z
                if (klat_map[end->x][end->y][cur->z].pi_z != cur->z)
                {
                    temp_z = klat_map[end->x][end->y][cur->z].pi_z;
                }

                // end is pin : break
                if (path_map[end->x][end->y].val == 2)
                {
                    // update capacity
                    update_cur_map_for_klat_xy(temp_z, &coor_array[cur->x][cur->y], &coor_array[end->x][end->y]);
                    two_pin_nets.emplace_back(coord_3d_map[cur->x][cur->y][temp_z], coord_3d_map[end->x][end->y][temp_z]);
                    q.push(&coord_3d_map[end->x][end->y][temp_z]);
                    continue;
                }

                // find segment's end
                while (1)
                {
                    // point has been visited
                    path_map[end->x][end->y].val = 0; // visited

                    int path = 0;
                    for (j = 0; j < 4; ++j)
                    {
                        path += path_map[end->x][end->y].edge[j];
                    }

                    // path > 1 : break
                    if (path > 1)
                    {

                        //  update capacity
                        update_cur_map_for_klat_xy(temp_z, &coor_array[cur->x][cur->y], &coor_array[end->x][end->y]);
                        two_pin_nets.emplace_back(coord_3d_map[cur->x][cur->y][temp_z], coord_3d_map[end->x][end->y][temp_z]);

                        q.push(&coord_3d_map[end->x][end->y][temp_z]);

                        break;
                    }

                    // next point
                    next->x = end->x + plane_dir[i][0];
                    next->y = end->y + plane_dir[i][1];

                    // std::cout << "while end " << end->x << " " << end->y << " " << temp_z << endl;
                    // std::cout << "while next " << next->x << " " << next->y << " " << temp_z << endl;
                    // std::cout << "val " << path_map[next->x][next->y].val << " pi_z " << klat_map[next->x][next->y][temp_z].pi_z;
                    // std::cout << " edge " << path_map[next->x][next->y].edge[i] << endl;

                    if (path_map[end->x][end->y].edge[i] != 1) // illegal
                    {

                        //  update capacity
                        update_cur_map_for_klat_xy(temp_z, &coor_array[cur->x][cur->y], &coor_array[end->x][end->y]);
                        two_pin_nets.emplace_back(coord_3d_map[cur->x][cur->y][temp_z], coord_3d_map[end->x][end->y][temp_z]);
                        q.push(&coord_3d_map[end->x][end->y][temp_z]);

                        break;
                    }
                    else // legal
                    {
                        // z direction change : break
                        if (klat_map[next->x][next->y][temp_z].pi_z != temp_z)
                        {
                            //  update capacity
                            update_cur_map_for_klat_xy(temp_z, &coor_array[cur->x][cur->y], &coor_array[end->x][end->y]);
                            two_pin_nets.emplace_back(coord_3d_map[cur->x][cur->y][temp_z], coord_3d_map[end->x][end->y][temp_z]);
                            q.push(&coord_3d_map[end->x][end->y][temp_z]);

                            break;
                        }

                        // next is pin : break
                        if (path_map[next->x][next->y].val == 2)
                        {
                            //  update capacity
                            update_cur_map_for_klat_xy(temp_z, &coor_array[cur->x][cur->y], &coor_array[next->x][next->y]);
                            two_pin_nets.emplace_back(coord_3d_map[cur->x][cur->y][temp_z], coord_3d_map[next->x][next->y][temp_z]);
                            q.push(&coord_3d_map[next->x][next->y][temp_z]);

                            break;
                        }
                    }

                    // shift one grid
                    end->x = next->x;
                    end->y = next->y;
                }
            }
        }

        // if (global_net_id == 1) // 51187
        //     std::cout << "pi_z " << klat_map[cur->x][cur->y][cur->z].pi_z << endl;
        // std::cout << "zmin zmax " << z_min << " " << z_max << endl;
        if (z_min != z_max)
        {
            update_cur_map_for_klat_z(z_min, z_max, &coor_array[cur->x][cur->y]);
            // std::cout << "coor 1 2 " << coord_3d_map[cur->x][cur->y][z_min].z << " " << coord_3d_map[cur->x][cur->y][z_max].z << endl;
            via.emplace_back(coord_3d_map[cur->x][cur->y][z_min], coord_3d_map[cur->x][cur->y][z_max]);
            // std::cout << "via 1 2 " << via.back().first.z << " " << via.back().second.z << endl;
            //  std::cout << "q pushhhhhhhhhhh " << next->x << " " << next->y << " " << next->z << endl;
        }

        // current point has been visited
        path_map[cur->x][cur->y].val = 0; // visited

        q.pop(); // dequeue
    }
    fout_tmp << rr_map->get_net_name(global_net_id) << "\n(\n";
    for (const auto &seg : two_pin_nets)
    {

        fout_tmp << rectToGuide(seg.first.x, seg.first.y, seg.second.x, seg.second.y, seg.first.z);
        // std::cout << seg.first.x << " " << seg.first.y << " " << seg.first.z << " | ";
        // std::cout << seg.second.x << " " << seg.second.y << " " << seg.second.z << endl;
    }

    if (pin_num != global_pin_num)
    {
      
        
        int pin_num = rr_map->get_netPinNumber(global_net_id);
        const PinptrList *pin_list = &rr_map->get_nPin(global_net_id);
        for (const auto &seg : via)
        {   
            z_max = seg.first.z;
            z_min = seg.second.z;

            // same (x,y) different z add via
            for (int k = 0; k < pin_num; k++)
            {
                if ((*pin_list)[k]->get_tileX() == seg.first.x && (*pin_list)[k]->get_tileY() == seg.first.y)
                {
                    //std::cout << "lost pin net add via " << netName << endl;
                    //std::cout << "pin " << start.x << " " << start.y << endl;
                    int layer = (*pin_list)[k]->get_layerId();
                    if (z_max < z_min)
                        swap(z_max, z_min);

                    z_max = max(z_max,layer);
                    z_min = min(z_min,layer);
                }
            }
            fout_tmp << viaToGuide(seg.first.x, seg.first.y, z_min, z_max);
        }
    }
    else
    {
        for (const auto &seg : via)
        {
            // std::cout << seg.first.x << " " << seg.first.y << " " << seg.first.z << " | ";
            // std::cout << seg.second.x << " " << seg.second.y << " " << seg.second.z << endl;
            fout_tmp << viaToGuide(seg.first.x, seg.first.y, seg.first.z, seg.second.z);
        }
    }
    fout_tmp << ")\n";
}

// check point if it is leaf node
// leaf node only has one edge which does not contain pin
void cycle_reduction(int x, int y)
{ // check point if it is leaf node
    int i, idx, temp_x, temp_y;

    for (i = 0; i < 4; ++i)
        if (path_map[x][y].edge[i] == 1)
            cycle_reduction(x + plane_dir[i][0], y + plane_dir[i][1]);

    // point does not have connnection point
    for (idx = 0; idx < 4 && path_map[x][y].edge[idx] == 0; ++idx)
        ;

    if (idx == 4 && path_map[x][y].val == 1) // a leaf && it is not a pin
    {
        path_map[x][y].val = 0;
        for (i = 0; i < 4; ++i)
        {
            temp_x = x + plane_dir[i][0];
            temp_y = y + plane_dir[i][1];
            if (temp_x >= 0 && temp_x < max_xx && temp_y >= 0 && temp_y < max_yy)
            {
                if (i == 0 && path_map[temp_x][temp_y].edge[1] == 1)
                    path_map[temp_x][temp_y].edge[1] = 0;
                else if (i == 1 && path_map[temp_x][temp_y].edge[0] == 1)
                    path_map[temp_x][temp_y].edge[0] = 0;
                else if (i == 2 && path_map[temp_x][temp_y].edge[3] == 1)
                    path_map[temp_x][temp_y].edge[3] = 0;
                else if (i == 3 && path_map[temp_x][temp_y].edge[2] == 1)
                    path_map[temp_x][temp_y].edge[2] = 0;
            }
        }
    }
}

int preprocess(int net_id)
{
    int i, k, x, y;
    int pin_layer;
    queue<Coordinate_2d *> q;
    Coordinate_2d *temp;
    const PinptrList *pin_list = &rr_map->get_nPin(net_id);
#ifdef POSTPROCESS
    int xy_len = 0;
#endif
    // after_xy_cost = 0;
    for (i = 0; i < global_pin_num; ++i)
    {
        x = (*pin_list)[i]->get_tileX();
        y = (*pin_list)[i]->get_tileY();
        pin_layer = (*pin_list)[i]->get_layerId();
        path_map[x][y].val = -2; // pin
        path_map[x][y].pin_layer = pin_layer;
    }

    // BFS speed up
    x = (*pin_list)[0]->get_tileX();
    y = (*pin_list)[0]->get_tileY();
    path_map[x][y].val = 2; // visited

    // initial klat_map[x][y][k]
    for (k = 0; k < max_zz; ++k)
    {
#ifdef DAC2023
        klat_map[x][y][k].val = INT_MAX;
#else
        klat_map[x][y][k].val = -1;
#endif
        klat_map[x][y][k].pi_z = -1;
    }
    q.push(&coor_array[x][y]); // enqueue
    while (!q.empty())
    {
        temp = q.front();
        for (i = 0; i < 4; ++i)
        {
            x = temp->x + plane_dir[i][0];
            y = temp->y + plane_dir[i][1];

            if (x >= 0 && x < max_xx && y >= 0 && y < max_yy && congestionMap2d->edge(temp->x, temp->y, i).lookupNet(net_id))
            {
                if (path_map[x][y].val == 0 || path_map[x][y].val == -2) // unvisited pin or node
                {
                    //++after_xy_cost;
#ifdef POSTPROCESS
                    ++xy_len;
#endif
                    global_BFS_xy++;
                    path_map[temp->x][temp->y].edge[i] = 1; // path direction?
                    if (path_map[x][y].val == 0)
                    {
                        path_map[x][y].val = 1; // visited node
                    }
                    else
                    {
                        path_map[x][y].val = 2; // visited pin
                    }
                    // initial klat_map[x][y][k]
                    for (k = 0; k < max_zz; ++k)
                    {
#ifdef DAC2023
                        klat_map[x][y][k].val = INT_MAX;
#else
                        klat_map[x][y][k].val = -1;
#endif
                        klat_map[x][y][k].pi_z = -1;
                    }
                    q.push(&coor_array[x][y]); // enqueue
                }
                else
                    path_map[temp->x][temp->y].edge[i] = 0; // can not pass
            }
            else
                path_map[temp->x][temp->y].edge[i] = 0; // can not pass
        }
        q.pop(); // dequeue
    }
    cycle_reduction((*pin_list)[0]->get_tileX(), (*pin_list)[0]->get_tileY());
#ifdef POSTPROCESS
    net_info[net_id].xy = xy_len;
#endif
#ifdef VIA_DENSITY
    return max_zz;
#else
    return max_zz; // max_layer; // modified
#endif
}

void PRINT_DP_INF(int level, int val, int *count_idx)
{
    int k, temp_x, temp_y, temp_val, temp_via_cost;
    int temp_idx[4];
    int min_k, max_k, temp_min_k, temp_max_k;

    if (level < 4)
    {
        for (k = 0; k < level; ++k)
            temp_idx[k] = count_idx[k];
        if (path_map[global_x][global_y].edge[level] == 1)
        {
            temp_x = global_x + plane_dir[level][0];
            temp_y = global_y + plane_dir[level][1];
            for (k = 0; k < global_max_layer; ++k) // use global_max_layer to substitue max_zz
                if (klat_map[temp_x][temp_y][k].val >= 0)
                {
                    std::cout << "x = " << temp_x << " y = " << temp_y << " k = " << k
                              << " val = " << klat_map[temp_x][temp_y][k].val << std::endl;
                    temp_idx[level] = k;
                    PRINT_DP_INF(level + 1, val + klat_map[temp_x][temp_y][k].val, temp_idx);
                }
        }
        else
        {
            temp_idx[level] = -1;
            PRINT_DP_INF(level + 1, val, temp_idx);
        }
    }
}

void rec_count(int level, int val, int *count_idx)
{
    int k, temp_x, temp_y, temp_val, temp_via_cost;
    int temp_idx[4];
    int min_k, max_k, temp_min_k, temp_max_k;

    if (level < 4)
    {
        if (val <= min_DP_val)
        {
            for (k = 0; k < level; ++k)
                temp_idx[k] = count_idx[k];
            if (path_map[global_x][global_y].edge[level] == 1)
            {
                temp_x = global_x + plane_dir[level][0];
                temp_y = global_y + plane_dir[level][1];
                for (k = 1; k < global_max_layer; ++k) // use global_max_layer to substitue max_zz
                    if (klat_map[temp_x][temp_y][k].val >= 0)
                    {
                        temp_idx[level] = k;
                        rec_count(level + 1, val + klat_map[temp_x][temp_y][k].val, temp_idx);
                    }
            }
            else
            {
                temp_idx[level] = -1;
                rec_count(level + 1, val, temp_idx);
            }
        }
    }
    else // level == 4
    {
        min_k = min_DP_k;
        max_k = max_DP_k;
        for (k = temp_via_cost = 0; k < 4; ++k)
            if (count_idx[k] != -1)
            {
                if (count_idx[k] < min_k)
                    min_k = count_idx[k];
                if (count_idx[k] > max_k)
                    max_k = count_idx[k];
                temp_via_cost +=
                    klat_map[global_x + plane_dir[k][0]][global_y + plane_dir[k][1]][count_idx[k]].via_cost;
            }

        temp_via_cost += ((max_k - min_k) * via_cost);

#ifdef VIA_DENSITY
        for (k = min_k, temp_val = val; k < max_k; ++k)
            if (viadensity_map[global_x][global_y][k].cur >= viadensity_map[global_x][global_y][k].max)
                ++temp_val;
#else
        temp_val = temp_via_cost;
#endif
        assert(temp_val >= 0);
        if (temp_val < min_DP_val)
        {
            min_DP_val = temp_val;
            min_DP_via_cost = temp_via_cost;
            for (k = 0; k < 4; ++k)
                min_DP_idx[k] = count_idx[k];
        }
        else if (temp_val == min_DP_val)
        {
#ifdef VIA_DENSITY
            if (temp_via_cost < min_DP_via_cost)
            {
                min_DP_via_cost = temp_via_cost;
                for (k = 0; k < 4; ++k)
                    min_DP_idx[k] = count_idx[k];
            }
            else if (temp_via_cost == min_DP_via_cost)
            {
                temp_min_k = min_DP_k;
                temp_max_k = max_DP_k;
                for (k = 0; k < 4; ++k)
                    if (min_DP_idx[k] != -1)
                    {
                        if (min_DP_idx[k] < temp_min_k)
                            temp_min_k = min_DP_idx[k];
                        if (min_DP_idx[k] > temp_max_k)
                            temp_max_k = min_DP_idx[k];
                    }
                if (max_k > temp_max_k || min_k > temp_min_k)
                    for (k = 0; k < 4; ++k)
                        min_DP_idx[k] = count_idx[k];
            }
#else
            temp_min_k = min_DP_k;
            temp_max_k = max_DP_k;
            for (k = 0; k < 4; ++k)
                if (min_DP_idx[k] != -1)
                {
                    if (min_DP_idx[k] < temp_min_k)
                        temp_min_k = min_DP_idx[k];
                    if (min_DP_idx[k] > temp_max_k)
                        temp_max_k = min_DP_idx[k];
                }
            if (max_k > temp_max_k || min_k > temp_min_k)
                for (k = 0; k < 4; ++k)
                    min_DP_idx[k] = count_idx[k];
#endif
        }
    }
}

void DP(int x, int y, int z)
{
    int i, k, temp_x, temp_y;
    bool is_end;
    int count_idx[4];
    int end_pin_layer;
    if (klat_map[x][y][z].val == -1) // non-traversed
    {
        is_end = true;
        for (i = 0; i < 4; ++i) // direction
        {
            if (path_map[x][y].edge[i] == 1) // check legal
            {
                is_end = false;
                temp_x = x + plane_dir[i][0];
                temp_y = y + plane_dir[i][1];

                bool assigned = false;
                if (*(overflow_map[x][y].edge[i]) <= 0) // this edge could not happen overflow
                {
                    for (k = 1; k < global_max_layer; ++k) // use global_max_layer to substitute max_zz
                    {
                        // pass without overflow
                        if (cur_map_3d[x][y][k].edge_list[i]->cur_cap < cur_map_3d[x][y][k].edge_list[i]->max_cap)
                        {
                            assigned = true;
                            DP(temp_x, temp_y, k);
                        } // modified
                    }
                }
                else // this edge could happen overflow
                {
                    for (k = 1; k < global_max_layer; ++k) // use global_max_layer to substitute max_zz
                    {
                        // overflow, but doesn't over the overflow_max
                        if ((cur_map_3d[x][y][k].edge_list[i]->cur_cap - cur_map_3d[x][y][k].edge_list[i]->max_cap) < overflow_max)
                        {
                            if ((i == 0 || i == 1) && /*k % 2 == 1*/ layerDirections[k] == 1)
                            {
                                DP(temp_x, temp_y, k);
                                assigned = true;
                            }
                            else if ((i == 2 || i == 3) && /*k % 2 == 0*/ layerDirections[k] == 0)
                            {
                                DP(temp_x, temp_y, k);
                                assigned = true;
                            }
                            // DP(temp_x, temp_y, k);
                        }
                    }
                }
                if (!assigned)
                {
                    std::cout << "Edge " << i << " of (" << x << ", " << y;
                    std::cout << "), overflow_map val = " << *(overflow_map[x][y].edge[i]) << std::endl;
                    std::cout << "2-D demand: " << congestionMap2d->edge(x, y, i).cur_cap << std::endl;
                    std::cout << "2-D max_cap: " << congestionMap2d->edge(x, y, i).max_cap << std::endl;
                    for (int k = 0; k < global_max_layer; ++k)
                    {
                        std::cout << "Layer " << k;
                        std::cout << ", cur_cap = " << cur_map_3d[x][y][k].edge_list[i]->cur_cap;
                        std::cout << ", max_cap = " << cur_map_3d[x][y][k].edge_list[i]->max_cap;
                        std::cout << std::endl;
                    }
                }
                assert(assigned);
            }
        }
        if (is_end == true)
        {

            end_pin_layer = path_map[x][y].pin_layer;
            // klat_map[x][y][z].via_cost = via_cost * z;
            if (z > end_pin_layer)
            {
                klat_map[x][y][z].via_cost = z - end_pin_layer;
                klat_map[x][y][z].val = klat_map[x][y][z].via_cost;
            }
            else
            { // # end_pin_layer >= z
                klat_map[x][y][z].via_cost = end_pin_layer - z;
                klat_map[x][y][z].val = klat_map[x][y][z].via_cost;
            }
        }
        else // is_end == false
        {
            // special purpose
            min_DP_val = 1000000000;
            min_DP_via_cost = 1000000000;
            global_x = x;
            global_y = y;
            max_DP_k = z;
            if (path_map[x][y].val == 1) // not a pin
                min_DP_k = z;
            else // pin
            {

                min_DP_k = path_map[x][y].pin_layer; // !!TODO
            }
            if (!is_metal5)
            {
                min_DP_k = min(min_DP_k, max_DP_k);
                max_DP_k = max(min_DP_k, max_DP_k);
            }

            for (i = 0; i < 4; ++i)
                min_DP_idx[i] = -1;

            assert(min_DP_k <= max_DP_k);
            rec_count(0, 0, count_idx);
            klat_map[x][y][z].via_cost = min_DP_via_cost;
            klat_map[x][y][z].val = min_DP_val;
            assert(klat_map[x][y][z].val >= 0);
            assert(klat_map[x][y][z].val < 1e9);

            for (i = 0; i < 4; ++i) // direction
            {
                if (min_DP_idx[i] > 0)
                {
                    temp_x = x + plane_dir[i][0];
                    temp_y = y + plane_dir[i][1];
                    klat_map[temp_x][temp_y][z].pi_z = min_DP_idx[i];
                }
            }
        }
    }
}

void klat(int net_id)
{
#ifdef PRINT_DP_PIN
    // get_netSerialNumber(int netId)
    // get_netId(int netSerial)
    if (net_id == 172)
    {
        check_net = true;
    }
    else
    {
        check_net = false;
    }

#endif

    const PinptrList *pin_list = &rr_map->get_nPin(net_id);
    Coordinate_2d *start;
    int start_pin_layer;

    start = &coor_array[(*pin_list)[0]->get_tileX()][(*pin_list)[0]->get_tileY()];
    start_pin_layer = (*pin_list)[0]->get_layerId();

#ifdef PRINT_DP_PIN
    if (check_net)
    {
        std::cout << "pin_num = " << pin_list->size() << std::endl;
        for (int i = 0; i < pin_list->size(); i++)
        {
            std::cout << i
                      << ":"
                         "x = "
                      << (*pin_list)[i]->get_tileX() << " y = " << (*pin_list)[i]->get_tileY()
                      << " z = " << (*pin_list)[i]->get_layerId() << std::endl;
        }
        const char *p;
        p = rr_map->get_netName(net_id);
        start_pin_layer std::cout << "net_id = " << net_id << "name = " << p << std::endl;
    }

#endif

    global_net_id = net_id; // LAZY global variable
    global_pin_num = rr_map->get_netPinNumber(net_id);
    global_max_layer = preprocess(net_id);
    // std::cout << "global_max_layer: " << global_max_layer << '\n';
    // fina a pin as starting point
    // klat start with a pin
#ifdef PIN_LEF
    auto dp_start = std::chrono::high_resolution_clock::now();
#ifdef DAC2023
    update(start->x, start->y);
#else
    DP(start->x, start->y, start_pin_layer);
#endif
    // if (CNT > 100) exit(0);
    // CNT++;
    auto dp_end = std::chrono::high_resolution_clock::now();
    auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(dp_end - dp_start);
    DP_TIME += time_span.count();
#else
    DP(start->x, start->y, 0);
#endif

#ifndef FROM2D
    if (temp_global_pin_cost < klat_map[start->x][start->y][0].val)
        if (temp_global_xy_cost <= after_xy_cost)
            printf("id %d, ori xy %d, ori z %d, curr xy %d, curr z %d\n", net_id, temp_global_xy_cost,
                   temp_global_pin_cost, after_xy_cost, klat_map[start->x][start->y][0].val);
#endif
#ifdef POSTPROCESS
    net_info[net_id].z = klat_map[start->x][start->y][0].via_cost;
    global_increase_vo = klat_map[start->x][start->y][0].val;
#endif
#ifdef PIN_LEF
    global_pin_cost += klat_map[start->x][start->y][start_pin_layer].val;
#else
    global_pin_cost += klat_map[start->x][start->y][0].val;
#endif
    update_path_for_klat(start, start_pin_layer);
    /*
    #ifdef PRINT_DP_PIN
        if(check_net){
            exit(0);
        }
    #endif
    */
}

int count_via_overflow_for_a_segment(int x, int y, int start, int end)
{
    int sum = 0, temp;

    if (start > end)
    {
        temp = start;
        start = end;
        end = temp;
    }
    while (start < end)
    {
        if (viadensity_map[x][y][start].cur >= viadensity_map[x][y][start].max)
            ++sum;
        ++start;
    }

    return sum;
}

void greedy(int net_id)
{
    const PinptrList *pin_list = &rr_map->get_nPin(net_id);
    Coordinate_2d *start;

    start = &coor_array[(*pin_list)[0]->get_tileX()][(*pin_list)[0]->get_tileY()];
    global_net_id = net_id; // LAZY global variable
    global_pin_num = rr_map->get_netPinNumber(net_id);
    preprocess(net_id);
}

void initial_BFS_color_map(void)
{
    int i, j, k;

    for (i = 0; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            for (k = 0; k < max_zz; ++k)
                BFS_color_map[i][j][k] = -1;
}

void malloc_BFS_color_map(void)
{
    int i, j;

    BFS_color_map = (int ***)malloc(sizeof(int **) * max_xx);
    for (i = 0; i < max_xx; ++i)
    {
        BFS_color_map[i] = (int **)malloc(sizeof(int *) * max_yy);
        for (j = 0; j < max_yy; ++j)
            BFS_color_map[i][j] = (int *)malloc(sizeof(int) * max_zz);
    }
}

void free_BFS_color_map(void)
{
    int i, j;

    for (i = 0; i < max_xx; ++i)
    {
        for (j = 0; j < max_yy; ++j)
            free(BFS_color_map[i][j]);
        free(BFS_color_map[i]);
    }
    free(BFS_color_map);
}

void calculate_wirelength(void)
{
    int i, j, k, xy = 0, z = 0;
    LRoutedNetTable::iterator iter;

    for (k = 0; k < max_zz; ++k)
    {
        for (i = 1; i < max_xx; ++i)
        {
            for (j = 0; j < max_yy; ++j)
            {
                xy += cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap;
            }
        }

        for (i = 0; i < max_xx; ++i)
        {
            for (j = 1; j < max_yy; ++j)
            {
                xy += cur_map_3d[i][j][k].edge_list[BACK]->cur_cap;
            }
        }
    }
    for (i = 0; i < max_xx; ++i)
    {
        for (j = 0; j < max_yy; ++j)
        {
            for (k = 1; k < max_zz; ++k)
            {
                z += (via_cost * cur_map_3d[i][j][k].edge_list[DOWN]->cur_cap);
            }
        }
    }

    printf("total wirelength = %d + %d = %d\n", xy, z, xy + z);
}

void erase_cur_map_3d(void)
{
    int i, j, k;

    for (i = 1; i < max_xx; ++i)
    {
        for (j = 0; j < max_yy; ++j)
        {
            for (k = 0; k < max_zz; ++k)
            {
                cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap = 0;
            }
        }
    }

    for (i = 0; i < max_xx; ++i)
    {
        for (j = 1; j < max_yy; ++j)
        {
            for (k = 0; k < max_zz; ++k)
            {
                cur_map_3d[i][j][k].edge_list[BACK]->cur_cap = 0;
            }
        }
    }
}

void multiply_viadensity_map_by_times(double times)
{
    int i, j, k;

    for (k = 1; k < max_zz; ++k)
    {
        double temp =
            (cur_map_3d[1][0][k - 1].edge_list[LEFT]->max_cap > 0)
                ? (cur_map_3d[1][0][k - 1].edge_list[LEFT]->max_cap * cur_map_3d[0][1][k].edge_list[BACK]->max_cap)
                : (cur_map_3d[1][0][k].edge_list[LEFT]->max_cap * cur_map_3d[0][1][k - 1].edge_list[BACK]->max_cap);
        temp = times * temp;
        temp /= 4.0;
        for (i = 0; i < max_xx; ++i)
            for (j = 0; j < max_yy; ++j)
            {
                viadensity_map[i][j][k - 1].max = (int)temp;
            }
    }
}

void sort_net_order(void)
{
    int i, max;

    LRoutedNetTable::iterator iter;

    clock_t start, finish;
#ifdef CHANGE_VIA_DENSITY
    double times;
#endif
    std::vector<int> temp_net_order;
    max = rr_map->get_netNumber();
    // re-disturibte net
    // multi_pin_net = (MULTIPIN_NET_NODE *)malloc(sizeof(MULTIPIN_NET_NODE) * max);
    // assert(multi_pin_net);
    // std::cout << "?\n";
    // 	find_group(max); // for comp_temp_net_order
    // std::cout << "?\n";

    initial_overflow_map();

    for (i = 0; i < max; ++i)
        temp_net_order.emplace_back(i);
    std::sort(temp_net_order.begin(), temp_net_order.end(), [&](auto a, auto b){
        return rr_map->get_netPinNumber(a) < rr_map->get_netPinNumber(b);
    });
    start = clock();

#ifdef CHANGE_VIA_DENSITY
    for (times = 0.8; times <= 1.2; times += 0.2)
    {
        initial_overflow_map();
        multiply_viadensity_map_by_times(times);
        for (k = 1; k < max_zz; ++k)
            for (i = 0; i < max_xx; ++i)
                for (j = 0; j < max_yy; ++j)
                    viadensity_map[i][j][k - 1].cur = 0;
#endif
#ifdef POSTPROCESS
        for (i = 0; i < max; ++i)
            net_info[i].xy = 0;
#endif
        for (i = 0; i < max; ++i)
        {
            if (temp_buf[3] == '0') // SOLA+APEC
            {
                if (temp_buf[2] == '1') // random
                    klat(i);
                else
                {
                    klat(temp_net_order[i]); // others
                }
            }
            else // greedy
            {
                if (temp_buf[2] == '1') // random
                    greedy(i);
                else
                    greedy(temp_net_order[i]); // others
            }
        }
#ifdef CHANGE_VIA_DENSITY
        print_via_overflow();
        calculate_wirelength();
        erase_cur_map_3d();
    }
#endif
    printf("cost = %lld\n", global_pin_cost);
    finish = clock();
    printf("time = %lf\n", (double)(finish - start) / CLOCKS_PER_SEC);

}

void calculate_cap(void)
{
    int i, j, overflow = 0, max = 0;

    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            if (congestionMap2d->edge(i, j, DIR_WEST).isOverflow())
            {
                overflow += (congestionMap2d->edge(i, j, DIR_WEST).overUsage()); // modified
                if (max < congestionMap2d->edge(i, j, DIR_WEST).overUsage())
                    max = congestionMap2d->edge(i, j, DIR_WEST).overUsage(); // modified
            }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j)
            if (congestionMap2d->edge(i, j, DIR_SOUTH).isOverflow())
            {
                overflow += (congestionMap2d->edge(i, j, DIR_SOUTH).overUsage()); // modified
                if (max < congestionMap2d->edge(i, j, DIR_SOUTH).overUsage())
                    max = congestionMap2d->edge(i, j, DIR_SOUTH).overUsage(); // modified
            }
    printf("2D overflow = %d\n", overflow);
    printf("2D max overflow = %d\n", max);
}

void generate_all_output(std::ofstream &fout)
{
    int i, max = rr_map->get_netNumber();

    // Local nets
    auto &deletedNet = rr_map->get_deleted_netList();
    for (auto &net : deletedNet)
    {
        // printf("%s\n(\n", net.get_name());
        fout_tmp << net.get_name() << "\n(\n";
        auto &pinList = net.get_pinList();
        int maxLayer = rr_map->get_layerNumber();
        int pin_cnt = 0;
        for (auto &pin : pinList)
        {
            ++pin_cnt;
            int x = pin->get_tileX();
            int y = pin->get_tileY();
            int layer = pin->get_layerId();

            fout_tmp << x * 4200 + 2100 << " " << y * 4200 + 2100 << " metal" << layer + 1 << " " << x * 4200 + 2100 << " " << y * 4200 + 2100 << " metal" << layer + 2 << '\n';
        }
        assert(pin_cnt <= 1);
        fout_tmp << ")\n";
    }
}

void Layer_assignment(const std::string &guide_path)
{
    assert(&congestionMap2d->edge(1, 1, DIR_NORTH) == &congestionMap2d->edge(1, 1, FRONT));
    assert(&congestionMap2d->edge(1, 1, DIR_SOUTH) == &congestionMap2d->edge(1, 1, BACK));
    assert(&congestionMap2d->edge(1, 1, DIR_EAST) == &congestionMap2d->edge(1, 1, RIGHT));
    assert(&congestionMap2d->edge(1, 1, DIR_WEST) == &congestionMap2d->edge(1, 1, LEFT));

    // std::string metal5 = TESTCASE_NAME.substr(TESTCASE_NAME.size()-6, 6);
    // is_metal5 = metal5 == "metal5";
    // std::cout << "Is metal5 testcases: " << is_metal5 << '\n';

    // string outputFileName(outputFileNamePtr);
    // int i;
    fout_tmp.open(guide_path);
    via_cost = 1;

    max_xx = rr_map->get_gridx();
    max_yy = rr_map->get_gridy();
    max_zz = rr_map->get_layerNumber();
    cout << max_xx << ' ' << max_yy << ' ' << max_zz << '\n';
    malloc_space();
#ifdef READ_OUTPUT
    printf("reading output...\n");
    // read_output();
    printf("reading output complete\n");
#endif
    calculate_cap();
    initial_3D_coordinate_map();
    find_overflow_max();
    puts("Layerassignment processing...");
#ifdef FROM2D
#ifdef POSTPROCESS
    net_info = (NET_INFO_NODE *)malloc(sizeof(NET_INFO_NODE) * rr_map->get_netNumber());
#endif
    sort_net_order();

    print_max_overflow();
#ifdef POSTPROCESS
    // refinement();
#endif
#endif
    puts("Layerassignment complete.");
#ifdef VIA_DENSITY
    // print_via_overflow();
#endif
    free_malloc_space();
    calculate_wirelength();
#ifdef PRINT_OVERFLOW
    print_max_overflow();
#endif
#ifdef ALLOUTPUT
    // printf("Outputing result file to %s\n", outputFileName.c_str());
    std::cout << "Outputing result file to " << guide_path << "\n";
    // std::ofstream fout(guide_path);
    malloc_BFS_color_map();
    initial_BFS_color_map();

    // int stdout_fd = dup(1);
    // FILE* outputFile = freopen(outputFileName.c_str(), "w", stdout);
    io_start = std::chrono::high_resolution_clock::now();
    generate_all_output(fout_tmp);
    io_end = std::chrono::high_resolution_clock::now();
    // fout.close();

    std::cout << "DP TIME: " << DP_TIME << '\n';
    //   fclose(outputFile);

    // stdout = fdopen(stdout_fd, "w");
    free_BFS_color_map();
#endif
#ifdef CHECK_PATH
    check_path();
#endif
}