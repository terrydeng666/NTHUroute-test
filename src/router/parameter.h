#pragma once

#include<string>
#include<unistd.h>
#include <vector>

enum {MIN_MAX_COST,
      MIN_TOTAL_COST};

enum {FASTROUTE_COST,
      OVERFLOW_COST,
      CONGESTION_COST,
      MADEOF_COST,
      HISTORY_COST,
      HISTORY_MADEOF_COST};

enum {NORMAL,
      WEIGHTED};

enum {MAZE_ROUTING_FASTROUTE_COST,
      MAZE_ROUTING_OVERFLOW_COST,
      MAZE_ROUTING_CONGESTION_COST,
      MAZE_ROUTING_MADEOF_COST};

enum {WINDOW_MODE1,
      WINDOW_MODE2,
      WINDOW_MODE3,
      WINDOW_MODE4,
      WINDOW_MODE5,
      WINDOW_MODE6,
      WINDOW_MODE7};

enum {O_EDGE_NUM, 
      T_OVERFLOW, 
      M_OVERFLOW, 
      B_SIZE};

enum {INC, 
      DEC};

class ParameterSet {
    public:
        //Construct_2d.cpp
        int pattern_route_obj;	//0:MIN_MAX_COST, 1:MIN_TOTAL_COST
        int pattern_route_cost;	//0:FASTROUTE_COST, 1:OVERFLOW_COST, 2:CONGESTION_COST
        int flute_mode;	        //0:NORMAL, 1:WEIGHTED
        int iter_2d;
        //Maze_routing_2d.cpp
        int maze_route_cost;	//0:MAZE_ROUTING_FASTROUTE_COST,
                                //1:MAZE_ROUTING_OVERFLOW_COST,
                                //2:MAZE_ROUTING_CONGESTION_COST
        int maze_size_mode;

        //Post_processing.cpp
        int maze_route_list_cost;	//O_EDGE_NUM, T_OVERFLOW, M_OVERFLOW, B_SIZE
        int maze_route_list_order;	//FRONT, BACK
        /* TroyLee: Parameter from command-line */
        int overflow_threshold;
        int iter_p3;
        std::string inputFileName;
        std::string outputFileName;
    public:
        ParameterSet(int pattern_route_obj = 0,
                     int pattern_route_cost = 0,
                     int flute_mode = 0,
                     int iter_2d = 0,
                     int maze_route_cost = 0,
                     int maze_size_mode = 0,
                     int maze_route_list_cost = 0,
                     int maze_route_list_order = 0,
                     int _overflow_threshold = 10,
                     int _iter_p3 = 10)
            :pattern_route_obj(pattern_route_obj),
             pattern_route_cost(pattern_route_cost),
             flute_mode(flute_mode),
             iter_2d(iter_2d),
             maze_route_cost(maze_route_cost),
             maze_size_mode(maze_size_mode),
             maze_route_list_cost(maze_route_list_cost),
             maze_route_list_order(maze_route_list_order),
             overflow_threshold(_overflow_threshold),
             iter_p3(_iter_p3)
        {}

        void    setSet ();

        void    setInputfile(std::string input);
        void    setOutputfile(std::string output);

        void    push_parameter(int pattern_route_obj = 0,
                               int pattern_route_cost = 0,
                               int flute_mode = 0,
                               int iter_2d = 0,
                               int maze_route_cost = 0,
                               int maze_size_mode = 0,
                               int maze_route_list_cost = 0,
                               int maze_route_list_order = 0,
                               int _overflow_threshold = 0,
                               int _iter_p3=10);
};

/* TroyLee: RouterParameter Fetch From Command-line */
/* Presetting Parameter Table */
struct PresettingParameters {
    /* Indentifier */
    int net_to_route;
    int grid_x, grid_y;
    /* Common Setting */
    bool monotonic_routing_en;
    /* Part 2 Setting */
    int  iteration_p2;
    int  init_box_size_p2;
    int  box_size_inc_p2;
    int  overflow_threshold;
    /* Part 3 Setting */
    int  iteration_p3;
    int  init_box_size_p3;
    int  box_size_inc_p3;
    /* Cost Function Selection */
    void (*cost_fp)(int i, int j, int dir);
};

class RoutingParameters {
    public:
        /* Init Parameters */
        RoutingParameters();
        ~RoutingParameters(){}

        void operator=( const struct PresettingParameters &preset );

        /* Setting Parameter */
        void set_monotonic_en(bool en);
        void set_simple_mode_en(bool en);
        void set_iteration_p2(int  it);
        void set_init_box_size_p2(int size);
        void set_box_size_inc_p2(int inc);
        void set_overflow_threshold(int th);

        void set_iteration_p3(int it);
        void set_init_box_size_p3(int size);
        void set_box_size_inc_p3(int inc);

        void set_rcong_min(double value);
        void set_rcong_diff(double value);
        void set_rcongV_min(double value);
        void set_rcongV_diff(double value);
        void set_rcongH_min(double value);
        void set_rcongH_diff(double value);
        void set_NonZeroV_min(double value);
        void set_NonZeroV_diff(double value);
        void set_NonZeroH_min(double value);
        void set_NonZeroH_diff(double value);
        void set_init_reduct(double value);

        /* Fetching Parameter */
        bool get_monotonic_en();
        bool get_simple_mode_en();
        int  get_iteration_p2();
        int  get_init_box_size_p2();
        int  get_box_size_inc_p2();
        int  get_overflow_threshold();

        int  get_iteration_p3();
        int  get_init_box_size_p3();
        int  get_box_size_inc_p3();

        double get_rcong_min();
        double get_rcong_diff();
        double get_rcongV_min();
        double get_rcongV_diff();
        double get_rcongH_min();
        double get_rcongH_diff();
        double get_NonZeroV_min();
        double get_NonZeroV_diff();
        double get_NonZeroH_min();
        double get_NonZeroH_diff();
        double get_init_reduct();


    private:
        /* Common Setting */
        bool monotonic_routing_en;
        bool simple_mode_en;

        /* Part 2 Setting */
        int  iteration_p2;
        int  init_box_size_p2;
        int  box_size_inc_p2;
        int  overflow_threshold;

        /* Part 3 Setting */
        int  iteration_p3;
        int  init_box_size_p3;
        int  box_size_inc_p3;

        /*GARY parameters*/
        double rcong_min;
        double rcong_diff;
        double rcongV_min;
        double rcongV_diff;
        double rcongH_min;
        double rcongH_diff;
        double rcongNonZeroV_min;
        double rcongNonZeroV_diff;
        double rcongNonZeroH_min;
        double rcongNonZeroH_diff;
        double init_reduct;
};

//{{{ *RoutingParameters* inline functions
inline void RoutingParameters::operator=( const struct PresettingParameters &preset )
{
    monotonic_routing_en = preset.monotonic_routing_en;
    iteration_p2         = preset.iteration_p2;
    init_box_size_p2     = preset.init_box_size_p2;
    box_size_inc_p2      = preset.box_size_inc_p2;
    overflow_threshold   = preset.overflow_threshold;
    iteration_p3         = preset.iteration_p3;
    init_box_size_p3     = preset.init_box_size_p3;
    box_size_inc_p3      = preset.box_size_inc_p3;
}

/* Setting Parameter */
inline 
void RoutingParameters::set_monotonic_en(bool en)
{
    monotonic_routing_en = en;
}

inline 
void RoutingParameters::set_simple_mode_en(bool en)
{
    simple_mode_en = en;
}

inline 
void RoutingParameters::set_iteration_p2(int  it)
{
    iteration_p2 = it;
}

inline 
void RoutingParameters::set_init_box_size_p2(int size)
{
    init_box_size_p2 = size;
}

inline 
void RoutingParameters::set_box_size_inc_p2(int inc)
{
    box_size_inc_p2 = inc;
}

inline 
void RoutingParameters::set_overflow_threshold(int th)
{
    overflow_threshold = th;
}

inline 
void RoutingParameters::set_iteration_p3(int it)
{
    iteration_p3 = it;
}

inline 
void RoutingParameters::set_init_box_size_p3(int size)
{
    init_box_size_p3 = size;
}

inline 
void RoutingParameters::set_box_size_inc_p3(int inc)
{
    box_size_inc_p3 = inc;
}

/* GARY parameters */
inline void RoutingParameters::set_rcong_min(double value) {
    rcong_min = value;
}
inline void RoutingParameters::set_rcong_diff(double value) {
    rcong_diff = value;
}
inline void RoutingParameters::set_rcongV_min(double value) {
    rcongV_min = value;
}
inline void RoutingParameters::set_rcongV_diff(double value) {
    rcongV_diff = value;
}
inline void RoutingParameters::set_rcongH_min(double value) {
    rcongH_min = value;
}
inline void RoutingParameters::set_rcongH_diff(double value) {
    rcongH_diff = value;
}
inline void RoutingParameters::set_NonZeroV_min(double value) {
    rcongNonZeroV_min = value;
}
inline void RoutingParameters::set_NonZeroV_diff(double value) {
    rcongNonZeroV_diff = value;
}
inline void RoutingParameters::set_NonZeroH_min(double value) {
    rcongNonZeroH_min = value;
}
inline void RoutingParameters::set_NonZeroH_diff(double value) {
    rcongNonZeroH_diff = value;
}
inline void RoutingParameters::set_init_reduct(double value) {
    init_reduct = value;
}

inline double RoutingParameters::get_rcong_min() {
    return rcong_min;
}
inline double RoutingParameters::get_rcong_diff() {
    return rcong_diff; 
}
inline double RoutingParameters::get_rcongV_min() {
    return rcongV_min;
}
inline double RoutingParameters::get_rcongV_diff() {
    return rcongV_diff;
}
inline double RoutingParameters::get_rcongH_min() {
    return rcongH_min; 
}
inline double RoutingParameters::get_rcongH_diff() {
    return rcongH_diff;
}
inline double RoutingParameters::get_NonZeroV_min() {
    return rcongNonZeroV_min;
}
inline double RoutingParameters::get_NonZeroV_diff() {
   return rcongNonZeroV_diff;
}
inline double RoutingParameters::get_NonZeroH_min() {
   return rcongNonZeroH_min;
}
inline double RoutingParameters::get_NonZeroH_diff() {
    return rcongNonZeroH_diff;
}
inline double RoutingParameters::get_init_reduct() {
    return init_reduct;
}

/* Fetching Parameter */

inline 
bool RoutingParameters::get_monotonic_en()
{
    return this->monotonic_routing_en;
}

inline 
bool RoutingParameters::get_simple_mode_en()
{
    return this->simple_mode_en;
}

inline 
int  RoutingParameters::get_iteration_p2()
{
    return this->iteration_p2;
}

inline 
int  RoutingParameters::get_init_box_size_p2()
{
    return this->init_box_size_p2;
}

inline 
int  RoutingParameters::get_box_size_inc_p2()
{
    return this->box_size_inc_p2;
}

inline 
int  RoutingParameters::get_overflow_threshold()
{
    return this->overflow_threshold;
}

inline 
int  RoutingParameters::get_iteration_p3()
{
    return this->iteration_p3;
}

inline 
int  RoutingParameters::get_init_box_size_p3()
{
    return this->init_box_size_p3;
}

inline 
int  RoutingParameters::get_box_size_inc_p3()
{
    return this->box_size_inc_p3;
}
//}}}
/* TroyLee: RouterParameter Fetch From Command-line */

class ParameterAnalyzer {
	public:
                ParameterAnalyzer(int argc, char* argv[]);
		//return input file name
		const char* input();
		//return output file name
		const char* output();
		//return parameter group NO.
        ParameterSet* parameter();
		//return the input type: 0 for IBM test cases  and 1 for the others
		int          caseType();
        RoutingParameters* routing_param();

        std::vector<std::string> get_lef_def();
        std::string& get_guide();
        const std::string& get_param();

        const std::string& get_cap_file(); // For ISPD 2024
        const std::string& get_net_file(); // For ISPD 2024
        const std::string& get_outPR_file(); // For ISPD 2024


	private:
		int         argc;
		char**      argv;
        std::string inputFileName;
        std::string outputFileName;
        std::string LefFileName;
        std::string DefFileName;
        std::string outputGuideName;
        std::string ParamFileName;
		int         paraNO; //parameter group NO.
		int         type;   //0 for IBM test cases  and 1 for the others
        ParameterSet parameterSet;
        RoutingParameters routingParam;

        std::string inputCapName; // For ISPD 2024
		std::string inputNetName; // For ISPD 2024
		std::string outputPRName; // For ISPD 2024

	private:
		void        analyze2();  //begin to analyze parameters
		void        analyzeInput();
		void        analyzeOutput();
};

inline
RoutingParameters*
ParameterAnalyzer::routing_param()
{
    return &routingParam;
}
inline 
std::vector<std::string>  
ParameterAnalyzer::get_lef_def(){
    std::vector<std::string> lef_def = {this->LefFileName,this->DefFileName};
    return  lef_def;
}
inline
std::string &
ParameterAnalyzer::get_guide()
{
    return outputGuideName;
}

inline const std::string& ParameterAnalyzer::get_param()
{
    return ParamFileName;
}

inline
const std::string &
ParameterAnalyzer::get_cap_file()
{
    return inputCapName;
}

inline
const std::string &
ParameterAnalyzer::get_net_file()
{
    return inputNetName;
}

inline
const std::string &
ParameterAnalyzer::get_outPR_file()
{
    return outputPRName;
}

