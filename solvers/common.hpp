#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace la {
using Cost = std::int64_t;
using Clock = std::chrono::steady_clock;
constexpr Cost INF = std::numeric_limits<Cost>::max()/4;
constexpr int NONE = -1;
struct Agent { int size, start, goal, heading; };
struct Options {
    std::string solver="mc-cbs", model="PM", policy="visit", objective="makespan";
    int max_horizon=-1;
    Cost max_soc=-1;
    double timeout=300, report=60;
    std::string progress_file;
    std::uint32_t seed=1;
    bool anytime=false;
    bool rm() const { return model=="RM"; }
    bool visit() const { return policy=="visit"; }
    bool soc() const { return objective=="soc"; }
};
struct Problem {
    int w=0,h=0;
    std::vector<std::string> grid;
    std::vector<Agent> agents;
    std::vector<int> clearance;
    static Problem read(std::istream& in) {
        Problem p; int n;
        if (!(in>>p.w>>p.h>>n) || p.w<1 || p.h<1 || 1LL*p.w*p.h>1000000 || n<0 || n>100000)
            throw std::runtime_error("invalid problem dimensions");
        p.grid.resize(p.h);
        for (auto& row:p.grid) {
            if (!(in>>row) || int(row.size())!=p.w || row.find_first_not_of(".#")!=std::string::npos)
                throw std::runtime_error("invalid map row");
        }
        p.make_clearance();
        for(int i=0;i<n;++i) {
            int s,x,y,gx,gy,d;
            if (!(in>>s>>x>>y>>gx>>gy>>d) || s<1 || !(s&1) || d<0 || d>3 ||
                !p.inside(x,y) || !p.inside(gx,gy))
                throw std::runtime_error("invalid agent");
            Agent a{s,y*p.w+x,gy*p.w+gx,d};
            if (!p.fits(a.start,s) || !p.fits(a.goal,s)) throw std::runtime_error("agent/goal intersects obstacle");
            for(const auto& b:p.agents) if(p.overlap(a.start,s,b.start,b.size))
                throw std::runtime_error("initial footprints overlap");
            p.agents.push_back(a);
        }
        return p;
    }
    bool inside(int x,int y)const{return x>=0&&y>=0&&x<w&&y<h;}
    bool fits(int v,int s)const{return v>=0&&v<w*h&&clearance[v]>=s;}
    bool overlap(int v,int s,int u,int z)const{
        return 2LL*std::abs(v%w-u%w)<Cost(s)+z && 2LL*std::abs(v/w-u/w)<Cost(s)+z;
    }
    bool swap(int i,int j,int a,int b,int na,int nb)const{
        return agents[i].size==1 && agents[j].size==1 && a!=b && a==nb && b==na;
    }
    int step(int v,int d)const{
        constexpr int dx[]={0,1,0,-1},dy[]={-1,0,1,0};
        int x=v%w+dx[d],y=v/w+dy[d];return inside(x,y)?y*w+x:NONE;
    }
    void make_clearance(){
        int W=w+2,H=h+2;
        std::vector<int> d(W*H,-1),q; q.reserve(d.size());
        for(int y=0;y<H;++y)for(int x=0;x<W;++x)
            if(!x||!y||x==W-1||y==H-1||grid[y-1][x-1]=='#'){
                int i=y*W+x;d[i]=0;q.push_back(i);
            }
        for(std::size_t k=0;k<q.size();++k){
            int v=q[k],x=v%W,y=v/W;
            for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
                int xx=x+dx,yy=y+dy;
                if(xx<0||yy<0||xx>=W||yy>=H)continue;
                int u=yy*W+xx;if(d[u]<0){d[u]=d[v]+1;q.push_back(u);}
            }
        }
        clearance.resize(w*h);
        for(int y=0;y<h;++y)for(int x=0;x<w;++x){
            int k=d[(y+1)*W+x+1];clearance[y*w+x]=k?2*k-1:0;
        }
    }
};
// Packed state: ((cell*4+heading)*2+completed). PM uses heading=0.
inline int state(int v,int d,bool b){return v*8+d*2+int(b);}
inline int cell(int s){return s/8;}
inline int dir(int s){return s/2%4;}
inline bool done(int s){return s&1;}
using Path=std::vector<int>;
using Paths=std::vector<Path>;
inline std::vector<int> initial(const Problem& p,const Options& o,int i){
    const auto& a=p.agents[i];int d=o.rm()?a.heading:0;
    if(a.start!=a.goal)return {state(a.start,d,false)};
    if(o.visit())return {state(a.start,d,true)};
    return {state(a.start,d,true),state(a.start,d,false)};
}
inline std::vector<int> next(const Problem& p,const Options& o,int i,int s){
    const auto& a=p.agents[i];int v=cell(s),d=dir(s);
    if(done(s)&&!o.visit())return {s};
    std::vector<int> out;
    auto add=[&](int u,int nd){
        if(!p.fits(u,a.size))return;
        if(done(s)||u!=a.goal)out.push_back(state(u,nd,done(s)));
        else {
            out.push_back(state(u,nd,true));
            if(!o.visit())out.push_back(state(u,nd,false));
        }
    };
    // Prefer waiting in cost-equivalent low-level paths after completion.
    add(v,d);
    if(o.rm()){
        add(p.step(v,d),d);add(v,(d+1)%4);add(v,(d+3)%4);
    }else for(int k=0;k<4;++k)add(p.step(v,k),0);
    return out;
}
struct Timeout {};
struct Stats {
    const Options& opt;
    Clock::time_point start=Clock::now();
    double last=0;
    std::uint64_t ct_generated=0,ct_expanded=0,ll_calls=0,ll_expanded=0,joint_generated=0,joint_expanded=0,open=0,polls=0;
    std::uint64_t lazy_attempts=0,lazy_failed=0,improvements=0;
    int horizon=-1;
    Cost lb=0,ub=INF;
    std::string phase="initialization";
    std::ofstream file;
    explicit Stats(const Options& o):opt(o){
        if(!o.progress_file.empty()){file.open(o.progress_file);if(!file)throw std::runtime_error("cannot open progress file");}
    }
    double elapsed()const{return std::chrono::duration<double>(Clock::now()-start).count();}
    void emit(){
        std::ostringstream s;
        s<<"{\"elapsed_sec\":"<<elapsed()<<",\"phase\":\""<<phase<<"\",\"horizon\":"<<horizon
          <<",\"lower_bound\":"<<lb<<",\"upper_bound\":";
        if(ub==INF)s<<"null";else s<<ub;
        s<<",\"ct_generated\":"<<ct_generated<<",\"ct_expanded\":"<<ct_expanded
         <<",\"low_level_calls\":"<<ll_calls<<",\"low_level_expanded\":"<<ll_expanded
         <<",\"joint_generated\":"<<joint_generated<<",\"joint_expanded\":"<<joint_expanded
         <<",\"lazy_attempts\":"<<lazy_attempts<<",\"lazy_failed\":"<<lazy_failed
         <<",\"improvements\":"<<improvements<<",\"open\":"<<open<<"}";
        std::cerr<<s.str()<<std::endl;
        if(file.is_open()){file<<s.str()<<std::endl;}
    }
    void tick(bool force=false){
        if(!force && (++polls&255)!=0)return;
        double e=elapsed();
        if(opt.report>0&&e-last>=opt.report){emit();last=e;}
        if(opt.timeout>0&&e>=opt.timeout)throw Timeout{};
    }
};
struct Distances {
    std::vector<std::vector<int>> d;
    explicit Distances(const Problem& p,const Options& o,Stats& st){
        // Reverse BFS on the individual orientation graph. Goals accept any heading.
        for(const auto& a:p.agents){
            st.tick(true);std::vector<int> ds(p.w*p.h*4,-1),q;
            for(int k=0;k<(o.rm()?4:1);++k){int v=a.goal*4+k;ds[v]=0;q.push_back(v);}
            for(std::size_t i=0;i<q.size();++i){
                st.tick();int u=q[i],v=u/4,k=u%4;
                auto add=[&](int v2,int k2){
                    if(!p.fits(v2,a.size))return;
                    int x=v2*4+k2;if(ds[x]<0){ds[x]=ds[u]+1;q.push_back(x);}
                };
                if(o.rm()){add(p.step(v,(k+2)%4),k);add(v,(k+1)%4);add(v,(k+3)%4);}
                else for(int j=0;j<4;++j)add(p.step(v,j),0);
            }
            d.push_back(std::move(ds));
        }
    }
    int get(int i,int s)const{return done(s)?0:d[i][s/2];}
};
inline std::pair<int,Cost> costs(const Paths& paths){
    int makespan=0;Cost soc=0;
    for(const auto& p:paths){
        auto it=std::find_if(p.begin(),p.end(),[](int s){return done(s);});
        if(it==p.end())throw std::logic_error("unfinished path");
        int t=int(it-p.begin());makespan=std::max(makespan,t);soc+=t;
    }
    return {makespan,soc};
}
struct Result {
    std::string status="TIMEOUT_NO_SOLUTION";
    Paths paths;
    bool has_solution=false;
};
inline void verify(const Problem& p,const Options& o,const Paths& paths){
    if(paths.size()!=p.agents.size())throw std::logic_error("wrong path count");
    if(paths.empty())return;
    std::size_t length=paths[0].size();
    if(!length)throw std::logic_error("empty path");
    for(int i=0;i<int(paths.size());++i){
        const auto& path=paths[i];
        auto roots=initial(p,o,i);
        if(path.size()!=length||std::find(roots.begin(),roots.end(),path.front())==roots.end()||!done(path.back()))
            throw std::logic_error("invalid endpoints");
        for(std::size_t t=1;t<length;++t){
            auto ns=next(p,o,i,path[t-1]);if(std::find(ns.begin(),ns.end(),path[t])==ns.end())
                throw std::logic_error("invalid individual transition");
        }
    }
    for(std::size_t t=0;t<length;++t)for(int i=0;i<int(paths.size());++i)for(int j=0;j<i;++j){
        int a=cell(paths[i][t]),b=cell(paths[j][t]);
        if(p.overlap(a,p.agents[i].size,b,p.agents[j].size)||
           (t&&p.swap(i,j,cell(paths[i][t-1]),cell(paths[j][t-1]),a,b)))
            throw std::logic_error("collision in solution");
    }
}
// Shared JSON trajectory serialization for final results and live checkpoints.
inline void write_paths(std::ostream& out,const Problem& p,const Paths& paths){
    out<<"[";
    for(std::size_t i=0;i<paths.size();++i){
        if(i)out<<",";
        out<<"[";
        for(std::size_t t=0;t<paths[i].size();++t){
            if(t)out<<",";
            int s=paths[i][t],v=cell(s);
            out<<"["<<v%p.w<<","<<v/p.w<<","<<dir(s)<<","<<int(done(s))<<"]";
        }
        out<<"]";
    }
    out<<"]";
}
inline std::string bounded_failure(const Options& o){
    if(o.max_horizon>=0&&o.max_soc>=0)return "INFEASIBLE_WITHIN_BOUNDS";
    if(o.max_horizon>=0)return "INFEASIBLE_WITHIN_HORIZON";
    if(o.max_soc>=0)return "INFEASIBLE_WITHIN_COST";
    return "UNSOLVABLE";
}
}
