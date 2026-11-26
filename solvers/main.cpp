#include "cbs.hpp"
#include "joint.hpp"
#include "lacam.hpp"
#include <cmath>
#include <cstdlib>

using namespace la;
int main(int argc,char** argv){
    try{
        Options o;
        for(int i=1;i<argc;++i){
            std::string k=argv[i];
            if(k=="--help"){
                std::cout<<"la_solver --solver mc-cbs|cbs|joint-bfs|lacam --model PM|RM --policy visit|stay "
                    "--objective makespan|soc --max-horizon N --max-soc N --timeout-sec S --report-interval-sec S "
                    "--seed N --progress-file PATH\nInput: width height agents, grid rows, then size sx sy gx gy heading per agent\n";
                return 0;
            }
            if(i+1==argc)throw std::runtime_error("missing value for "+k);
            std::string v=argv[++i];
            if(k=="--solver")o.solver=v;
            else if(k=="--model")o.model=v;
            else if(k=="--policy")o.policy=v;
            else if(k=="--objective")o.objective=v;
            else if(k=="--max-horizon")o.max_horizon=std::stoi(v);
            else if(k=="--max-soc")o.max_soc=std::stoll(v);
            else if(k=="--timeout-sec")o.timeout=std::stod(v);
            else if(k=="--report-interval-sec")o.report=std::stod(v);
            else if(k=="--seed"){auto seed=std::stoull(v);if(seed>std::numeric_limits<std::uint32_t>::max())throw std::runtime_error("invalid seed");o.seed=std::uint32_t(seed);}
            else if(k=="--progress-file")o.progress_file=v;
            else throw std::runtime_error("unknown option "+k);
        }
        if(o.solver!="mc-cbs"&&o.solver!="cbs"&&o.solver!="joint-bfs"&&o.solver!="lacam")throw std::runtime_error("unknown solver");
        if(o.model!="PM"&&o.model!="RM")throw std::runtime_error("unknown model");
        if(o.policy!="stay"&&o.policy!="visit")throw std::runtime_error("unknown policy");
        if(o.objective!="makespan"&&o.objective!="soc")throw std::runtime_error("unknown objective");
        if(o.max_horizon< -1||o.max_horizon==std::numeric_limits<int>::max()||
           o.max_soc< -1||o.max_soc>=std::numeric_limits<int>::max()||
           !std::isfinite(o.timeout)||!std::isfinite(o.report)||o.timeout<0||o.report<0)
            throw std::runtime_error("invalid search bounds");
        Problem p=Problem::read(std::cin);
        Stats st(o);Result result;
        try{
            if(o.solver=="joint-bfs")result=joint_search(p,o,st);
            else if(o.solver=="lacam")lazy_search(p,o,st,result);
            else cbs_search(p,o,st,result);
        }catch(const Timeout&){
            result.status=result.has_solution?"TIMEOUT_WITH_SOLUTION":"TIMEOUT_NO_SOLUTION";
        }catch(const std::bad_alloc&){
            result.status=result.has_solution?"RESOURCE_LIMIT_WITH_SOLUTION":"RESOURCE_LIMIT_NO_SOLUTION";
        }
        int makespan=0;Cost soc=0;
        if(result.has_solution){
            verify(p,o,result.paths);
            std::tie(makespan,soc)=costs(result.paths);
            if((o.max_horizon>=0&&makespan>o.max_horizon)||(o.max_soc>=0&&soc>o.max_soc))
                throw std::logic_error("solution violates bounds");
        }
        st.phase="finished";st.emit();
        std::cout<<"{\"version\":1,\"status\":\""<<result.status<<"\",\"solver\":\""<<o.solver
            <<"\",\"motion_model\":\""<<o.model<<"\",\"goal_policy\":\""<<o.policy
            <<"\",\"objective\":\""<<o.objective<<"\",\"max_horizon\":"<<o.max_horizon
            <<",\"max_soc\":"<<o.max_soc<<",\"lower_bound\":"<<st.lb<<",\"upper_bound\":";
        if(st.ub==INF)std::cout<<"null";else std::cout<<st.ub;
        std::cout<<",\"elapsed_sec\":"<<st.elapsed()<<",\"ct_expanded\":"<<st.ct_expanded
            <<",\"ct_generated\":"<<st.ct_generated<<",\"low_level_calls\":"<<st.ll_calls
            <<",\"low_level_expanded\":"<<st.ll_expanded<<",\"joint_generated\":"<<st.joint_generated
            <<",\"joint_expanded\":"<<st.joint_expanded<<",\"lazy_attempts\":"<<st.lazy_attempts
            <<",\"lazy_failed\":"<<st.lazy_failed<<",\"seed\":"<<o.seed<<",\"makespan\":";
        if(result.has_solution)std::cout<<makespan;else std::cout<<"null";
        std::cout<<",\"soc\":";
        if(result.has_solution)std::cout<<soc;else std::cout<<"null";
        std::cout<<",\"paths\":";
        if(!result.has_solution)std::cout<<"null";else{
            std::cout<<"[";
            for(int i=0;i<int(result.paths.size());++i){
                if(i)std::cout<<",";
                std::cout<<"[";
                for(std::size_t t=0;t<result.paths[i].size();++t){
                    if(t)std::cout<<",";
                    int s=result.paths[i][t],v=cell(s);
                    std::cout<<"["<<v%p.w<<","<<v/p.w<<","<<dir(s)<<","<<(done(s)?1:0)<<"]";
                }
                std::cout<<"]";
            }
            std::cout<<"]";
        }
        std::cout<<"}\n";return 0;
    }catch(const std::exception& e){std::cerr<<"error: "<<e.what()<<std::endl;return 2;}
}
