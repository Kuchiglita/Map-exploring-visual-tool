#pragma once
#include "common.hpp"

namespace la {
struct VecHash {
    std::size_t operator()(const std::vector<int>& v)const{
        std::size_t h=0;for(int x:v)h^=std::hash<int>{}(x)+0x9e3779b9+(h<<6)+(h>>2);return h;
    }
};
struct JointNode {
    std::vector<int> config;
    int t;Cost soc;std::size_t parent;
};
inline Result joint_search(const Problem& p,const Options& o,Stats& st){
    // Unit-cost BFS for makespan (SOC breaks ties); Dijkstra for SOC.
    // When a horizon matters, time is part of the key. Otherwise a label
    // for the same configuration dominates every worse lexicographic label.
    st.phase="joint";
    const int n=int(p.agents.size());
    int cap=o.max_horizon;
    if(o.max_soc>=0 && o.max_soc<=std::numeric_limits<int>::max())
        cap=cap<0?int(o.max_soc):std::min(cap,int(o.max_soc));
    const bool timed=cap>=0;
    using Item=std::tuple<Cost,Cost,std::size_t>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    std::vector<JointNode> nodes;
    std::unordered_map<std::vector<int>,std::size_t,VecHash> best;
    const auto sentinel=std::numeric_limits<std::size_t>::max();
    auto key=[&](const std::vector<int>& c,int t){auto k=c;if(timed)k.push_back(t);return k;};
    auto priority=[&](int t,Cost soc){return o.soc()?std::pair<Cost,Cost>{soc,t}:std::pair<Cost,Cost>{t,soc};};
    auto push=[&](const std::vector<int>& c,int t,Cost soc,std::size_t parent){
        if(o.max_soc>=0&&soc>o.max_soc)return;
        auto k=key(c,t);auto it=best.find(k);
        auto pr=priority(t,soc);
        if(it!=best.end()&&priority(nodes[it->second].t,nodes[it->second].soc)<=pr)return;
        std::size_t id=nodes.size();nodes.push_back({c,t,soc,parent});best[std::move(k)]=id;
        queue.emplace(pr.first,pr.second,id);++st.joint_generated;
    };
    std::vector<int> temp(n);
    std::function<void(int)> roots=[&](int i){
        st.tick();
        if(i==n){push(temp,0,0,sentinel);return;}
        for(int s:initial(p,o,i)){temp[i]=s;roots(i+1);}
    };
    roots(0);
    while(!queue.empty()){
        auto [value,secondary,id]=queue.top();queue.pop();
        auto node=nodes[id]; // not a reference: successor insertion can reallocate.
        auto it=best.find(key(node.config,node.t));
        if(it==best.end()||it->second!=id)continue;
        st.lb=std::max(st.lb,value);st.open=queue.size();++st.joint_expanded;st.tick();
        bool completed=std::all_of(node.config.begin(),node.config.end(),[](int s){return done(s);});
        if(completed){
            Paths paths(n);
            for(std::size_t k=id;k!=sentinel;k=nodes[k].parent)
                for(int i=0;i<n;++i)paths[i].push_back(nodes[k].config[i]);
            for(auto& path:paths)std::reverse(path.begin(),path.end());
            st.ub=value;return {"OPTIMAL",std::move(paths),true};
        }
        if(timed&&node.t>=cap)continue;
        if(node.t==std::numeric_limits<int>::max())throw std::runtime_error("joint time exceeds implementation limit");
        Cost new_soc=node.soc;
        for(int s:node.config)new_soc+=!done(s);
        if(o.max_soc>=0&&new_soc>o.max_soc)continue;
        std::function<void(int)> gen=[&](int i){
            st.tick();
            if(i==n){push(temp,node.t+1,new_soc,id);return;}
            for(int s:next(p,o,i,node.config[i])){
                bool legal=true;
                for(int j=0;j<i;++j){
                    if(p.overlap(cell(s),p.agents[i].size,cell(temp[j]),p.agents[j].size)||
                       p.swap(i,j,cell(node.config[i]),cell(node.config[j]),cell(s),cell(temp[j]))){legal=false;break;}
                }
                if(legal){temp[i]=s;gen(i+1);}
            }
        };
        gen(0);
    }
    return {bounded_failure(o),{},false};
}
}
