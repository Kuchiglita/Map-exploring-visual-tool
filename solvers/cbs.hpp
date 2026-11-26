#pragma once
#include "common.hpp"
#include <memory>
#include <unordered_set>

namespace la {
struct Constraint {
    int t,kind,x0,y0,x1,y1,from,to;
    static Constraint vertex(int t,int x0,int y0,int x1,int y1){
        return {t,0,x0,y0,x1,y1,-1,-1};
    }
    static Constraint edge(int t,int from,int to){
        return {t,1,0,0,0,0,from,to};
    }
    auto tuple()const{return std::tie(t,kind,x0,y0,x1,y1,from,to);}
    bool operator<(const Constraint& b)const{return tuple()<b.tuple();}
    bool forbidden(int u,int v,int w)const{
        if(kind==1)return u==from&&v==to;
        int x=v%w,y=v/w;return x>=x0&&x<=x1&&y>=y0&&y<=y1;
    }
};
using Restrictions=std::vector<std::vector<Constraint>>;
struct LLRec { int g;std::uint64_t parent; };
inline std::optional<Path> low_level(const Problem& p,const Options& o,const Distances& d,
    Stats& st,int agent,int H,const std::vector<Constraint>& constraints){
    ++st.ll_calls;st.tick(true);
    std::unordered_map<int,std::vector<Constraint>> by_time;
    for(const auto& c:constraints)by_time[c.t].push_back(c);
    auto allowed=[&](int t,int u,int v){
        auto it=by_time.find(t);if(it==by_time.end())return true;
        for(const auto& c:it->second)if(c.forbidden(u,v,p.w))return false;
        return true;
    };
    auto key=[](int t,int s){return (std::uint64_t(t)<<32)|std::uint32_t(s);};
    const auto sentinel=std::numeric_limits<std::uint64_t>::max();
    using Item=std::tuple<int,int,std::uint64_t,int,std::uint64_t>;
    std::uint64_t serial=0;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;
    std::unordered_map<std::uint64_t,LLRec> rec;
    auto push=[&](int t,int s,int g,std::uint64_t parent){
        int h=d.get(agent,s);
        if(h<0||h>H-t||(o.max_soc>=0&&Cost(g)+h>o.max_soc))return;
        auto k=key(t,s);auto it=rec.find(k);if(it!=rec.end()&&it->second.g<=g)return;
        rec[k]={g,parent};open.emplace(g+h,-t,serial++,g,k);
    };
    for(int s:initial(p,o,agent))if(allowed(0,-1,cell(s)))push(0,s,0,sentinel);
    while(!open.empty()){
        auto [f,nt,order,g,k]=open.top();open.pop();int t=-nt,s=int(std::uint32_t(k));
        auto it=rec.find(k);if(it==rec.end()||it->second.g!=g)continue;
        ++st.ll_expanded;st.tick();
        if(t==H){
            if(!done(s))continue;
            Path path;
            for(auto cur=k;cur!=sentinel;cur=rec.at(cur).parent)path.push_back(int(std::uint32_t(cur)));
            std::reverse(path.begin(),path.end());return path;
        }
        for(int ns:next(p,o,agent,s))
            if(allowed(t+1,cell(s),cell(ns)))push(t+1,ns,g+!done(s),k);
    }
    return std::nullopt;
}
struct Conflict { int i,j,t;bool edge;int a,b,pa,pb; };
inline std::optional<Conflict> conflict(const Problem& p,const Paths& paths,Stats& st){
    if(paths.empty())return std::nullopt;
    for(int t=0;t<int(paths[0].size());++t)
        for(int i=0;i<int(paths.size());++i)for(int j=0;j<i;++j){
            st.tick();
            int a=cell(paths[i][t]),b=cell(paths[j][t]);
            if(p.overlap(a,p.agents[i].size,b,p.agents[j].size))
                return Conflict{i,j,t,false,a,b,-1,-1};
            if(t){
                int pa=cell(paths[i][t-1]),pb=cell(paths[j][t-1]);
                if(p.swap(i,j,pa,pb,a,b))return Conflict{i,j,t,true,a,b,pa,pb};
            }
        }
    return std::nullopt;
}
inline std::string constraint_key(const Restrictions& c){
    std::ostringstream s;
    for(int i=0;i<int(c.size());++i){
        s<<i<<":";
        for(const auto& x:c[i])s<<x.t<<","<<x.kind<<","<<x.x0<<","<<x.y0<<","<<x.x1<<","<<x.y1<<","<<x.from<<","<<x.to<<";";
        s<<"|";
    }
    return s.str();
}
struct CTNode {
    Restrictions constraints;
    Paths paths;
    Cost cost=0;
    std::uint64_t serial=0,depth=0;
};
struct CTGreater {
    bool operator()(const std::shared_ptr<CTNode>& a,const std::shared_ptr<CTNode>& b)const{
        if(a->cost!=b->cost)return a->cost>b->cost;
        if(a->depth!=b->depth)return a->depth<b->depth;
        return a->serial>b->serial;
    }
};
inline std::optional<Paths> fixed_horizon(const Problem& p,const Options& o,const Distances& d,
    Stats& st,int H,bool soc_bound){
    st.horizon=H;st.phase="fixed_horizon";st.open=0;st.tick(true);
    int n=int(p.agents.size());
    auto root=std::make_shared<CTNode>();root->constraints.resize(n);root->paths.resize(n);
    for(int i=0;i<n;++i){
        auto path=low_level(p,o,d,st,i,H,root->constraints[i]);
        if(!path)return std::nullopt;
        root->paths[i]=std::move(*path);
    }
    root->cost=costs(root->paths).second;
    if(o.max_soc>=0&&root->cost>o.max_soc)return std::nullopt;
    std::priority_queue<std::shared_ptr<CTNode>,std::vector<std::shared_ptr<CTNode>>,CTGreater> open;
    std::unordered_set<std::string> seen;
    seen.insert(constraint_key(root->constraints));open.push(root);++st.ct_generated;
    while(!open.empty()){
        auto node=open.top();open.pop();++st.ct_expanded;
        st.open=open.size();
        if(soc_bound)st.lb=std::max(st.lb,std::min(node->cost,st.ub));
        st.tick(true);
        auto c=conflict(p,node->paths,st);
        if(!c)return std::move(node->paths);
        auto vtx=[&](int pos,int radius){
            int x=pos%p.w,y=pos/p.w;
            return Constraint::vertex(c->t,std::max(0,x-radius),std::max(0,y-radius),
                std::min(p.w-1,x+radius),std::min(p.h-1,y+radius));
        };
        // ASYM: any valid solution either avoids A's exact center or
        // avoids every B center that overlaps A's exact center.
        int rad=(p.agents[c->i].size+p.agents[c->j].size)/2-1;
        std::array<std::pair<int,Constraint>,2> branches={
            std::make_pair(c->i,c->edge?Constraint::edge(c->t,c->pa,c->a):vtx(c->a,0)),
            std::make_pair(c->j,c->edge?Constraint::edge(c->t,c->pb,c->b):
                (o.solver=="mc-cbs"?vtx(c->a,rad):vtx(c->b,0)))
        };
        for(auto [i,restriction]:branches){
            st.tick(true);
            auto child=std::make_shared<CTNode>(*node);
            auto& cs=child->constraints[i];
            cs.insert(std::lower_bound(cs.begin(),cs.end(),restriction),restriction);
            if(!seen.insert(constraint_key(child->constraints)).second)continue;
            auto path=low_level(p,o,d,st,i,H,cs);
            if(!path)continue;
            child->paths[i]=std::move(*path);
            child->cost=costs(child->paths).second;
            if(o.max_soc>=0&&child->cost>o.max_soc)continue;
            child->depth=node->depth+1;child->serial=++st.ct_generated;open.push(std::move(child));
        }
        st.open=open.size();
    }
    return std::nullopt;
}
// Number of augmented joint configurations is bounded by product of
// individual state counts. A shortest feasible completion never repeats
// an augmented configuration (remove the loop). Also holds with SOC cap.
inline std::optional<int> finite_bound(const Problem& p,const Options& o,Stats& st){
    Cost product=1;
    constexpr Cost cap=std::numeric_limits<int>::max()/2;
    for(const auto& a:p.agents){
        Cost n=0;
        for(int c:p.clearance){st.tick();n+=c>=a.size;}
        n*=o.rm()?8:2;
        if(n==0)return 0;
        if(product>cap/n)return std::nullopt;
        product*=n;
    }
    return int(product-1);
}
inline void store_solution(Result& result,Paths paths,const Options& o,Stats& st){
    auto [makespan,soc]=costs(paths);
    Cost value=o.soc()?soc:makespan;
    if(!result.has_solution||value<st.ub){
        for(auto& path:paths)path.resize(std::size_t(makespan)+1);
        result.paths=std::move(paths);result.has_solution=true;st.ub=value;
    }
}
inline void cbs_search(const Problem& p,const Options& o,Stats& st,Result& result){
    if(p.agents.empty()){result={"OPTIMAL",{},true};st.lb=st.ub=0;return;}
    Distances d(p,o,st);
    int initialH=0;Cost initialSOC=0;
    for(int i=0;i<int(p.agents.size());++i){
        int h=d.get(i,initial(p,o,i).front());
        if(h<0){result.status="UNSOLVABLE";return;}
        initialH=std::max(initialH,h);initialSOC+=h;
    }
    st.lb=o.soc()?initialSOC:initialH;
    if(o.max_soc>=0&&initialSOC>o.max_soc){
        result.status=bounded_failure(o);return;
    }
    int cap=o.max_horizon;
    if(o.max_soc>=0)cap=cap<0?int(o.max_soc):std::min(cap,int(o.max_soc));
    auto global=finite_bound(p,o,st);
    if(global)cap=cap<0?*global:std::min(cap,*global);
    if(cap>=0&&cap<initialH){result.status=bounded_failure(o);return;}
    // For bounded SOC, the whole relevant horizon is available at once.
    // With no explicit bounds, first find a feasible incumbent and then
    // prove global optimality at H <= incumbent SOC (or the finite state bound).
    if(o.soc()&&(o.max_horizon>=0||o.max_soc>=0)){
        auto sol=fixed_horizon(p,o,d,st,cap,true);
        if(!sol){result.status=bounded_failure(o);return;}
        store_solution(result,std::move(*sol),o,st);
        st.lb=st.ub;result.status="OPTIMAL";return;
    }
    for(int H=initialH;;++H){
        if(cap>=0&&H>cap){
            result.status=bounded_failure(o);return;
        }
        if(!o.soc())st.lb=H;
        auto sol=fixed_horizon(p,o,d,st,H,false);
        if(sol){
            store_solution(result,std::move(*sol),o,st);
            if(!o.soc()||st.lb==st.ub||H>=st.ub||(global&&H>=*global)){
                st.lb=st.ub;result.status="OPTIMAL";return;
            }
            Cost proofH=global?std::min<Cost>(*global,st.ub):st.ub;
            if(proofH>std::numeric_limits<int>::max()-1)
                throw std::runtime_error("SOC proof horizon exceeds supported integer range; specify bounds");
            auto proof=fixed_horizon(p,o,d,st,int(proofH),true);
            if(!proof)throw std::logic_error("known incumbent lost at proof horizon");
            store_solution(result,std::move(*proof),o,st);
            st.lb=st.ub;result.status="OPTIMAL";return;
        }
        if(!o.soc())st.lb=Cost(H)+1;
        if(H==std::numeric_limits<int>::max()-1)
            throw std::runtime_error("search horizon exceeds supported integer range");
    }
}
}
