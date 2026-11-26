#pragma once
#include "joint.hpp"
#include <deque>
#include <memory>
#include <numeric>
#include <random>

namespace la {
// LaCAM-style lazy constraint addition, adapted to the exact discrete
// LA model. Each high-level node owns a queue of partial assignments
// for the next configuration. PIBT-like inheritance is only a guide;
// fully specified assignments are eventually enumerated independently.
struct LazyPrefix {
    std::vector<int> states; // positions in the node's fixed priority order
};
struct LazyNode {
    std::vector<int> config, order;
    std::vector<std::vector<int>> moves;
    std::deque<LazyPrefix> pending;
    std::size_t parent;
    int t;
    Cost soc, heuristic;
    std::uint64_t attempts=0;
    bool prepared=false;
    LazyNode(std::vector<int> c,std::size_t p,int time,Cost cost,Cost h):
        config(std::move(c)),parent(p),t(time),soc(cost),heuristic(h){}
};
inline bool lazy_conflict(const Problem& p,const std::vector<int>& cur,int i,int s,int j,int u){
    return p.overlap(cell(s),p.agents[i].size,cell(u),p.agents[j].size)||
        p.swap(i,j,cell(cur[i]),cell(cur[j]),cell(s),cell(u));
}
inline void lazy_prepare(LazyNode& n,const Problem& p,const Options& o,
                         const Distances& d,Stats& st,std::mt19937& rng){
    const int N=int(n.config.size());
    n.order.resize(N);std::iota(n.order.begin(),n.order.end(),0);
    std::shuffle(n.order.begin(),n.order.end(),rng);
    // Larger unfinished agents are harder to route; finished Visit agents
    // are low priority but can still inherit priority and move aside.
    std::stable_sort(n.order.begin(),n.order.end(),[&](int i,int j){
        return std::make_tuple(!done(n.config[i]),p.agents[i].size,d.get(i,n.config[i]))>
               std::make_tuple(!done(n.config[j]),p.agents[j].size,d.get(j,n.config[j]));
    });
    n.moves.resize(N);
    for(int i=0;i<N;++i){
        st.tick();
        auto& m=n.moves[i];m=next(p,o,i,n.config[i]);
        std::shuffle(m.begin(),m.end(),rng);
        std::stable_sort(m.begin(),m.end(),[&](int a,int b){
            // After completing a Visit, prefer waiting unless displaced.
            auto rank=[&](int s){
                return std::make_tuple(d.get(i,s)<0?std::numeric_limits<int>::max():d.get(i,s),
                    done(n.config[i])&&s!=n.config[i],s==n.config[i]&&!done(s),!done(s));
            };
            return rank(a)<rank(b);
        });
    }
    n.pending.push_back({{}});n.prepared=true;
}
inline std::optional<std::vector<int>> lazy_generate(const Problem& p,const LazyNode& n,
    const LazyPrefix& prefix,Stats& st){
    const int N=int(n.config.size());
    std::vector<int> out(N,NONE),trail;
    auto compatible=[&](int i,int s){
        for(int j=0;j<N;++j){
            st.tick();
            if(j!=i&&out[j]!=NONE&&lazy_conflict(p,n.config,i,s,j,out[j]))return false;
        }
        return true;
    };
    for(std::size_t k=0;k<prefix.states.size();++k){
        int i=n.order[k],s=prefix.states[k];
        if(!compatible(i,s))return std::nullopt;
        out[i]=s;
    }
    // Forced assignments can displace several heterogeneous agents. The
    // recursive trail is rolled back on failure, never changing a prefix.
    std::function<bool(int)> assign=[&](int i){
        st.tick();
        if(out[i]!=NONE)return true;
        for(int s:n.moves[i]){
            if(!compatible(i,s))continue;
            auto mark=trail.size();
            out[i]=s;trail.push_back(i);
            bool ok=true;
            for(int j:n.order){
                st.tick();
                if(out[j]==NONE&&p.overlap(cell(s),p.agents[i].size,cell(n.config[j]),p.agents[j].size)
                   && !assign(j)){ok=false;break;}
            }
            if(ok)return true;
            while(trail.size()>mark){out[trail.back()]=NONE;trail.pop_back();}
        }
        return false;
    };
    // First let every forced move inherit priority for the agents it
    // displaces. Then assign the remaining agents in priority order.
    for(std::size_t k=0;k<prefix.states.size();++k){
        int i=n.order[k];
        for(int j:n.order){
            st.tick();
            if(out[j]==NONE&&p.overlap(cell(out[i]),p.agents[i].size,cell(n.config[j]),p.agents[j].size)
               && !assign(j))return std::nullopt;
        }
    }
    for(int i:n.order)if(!assign(i))return std::nullopt;
    return out;
}
inline void lazy_search(const Problem& p,const Options& o,Stats& st,Result& result){
    st.phase="lacam_initialization";
    const int N=int(p.agents.size());
    if(!N){result={"OPTIMAL",{},true};st.lb=st.ub=0;return;}
    Distances d(p,o,st);
    const auto sentinel=std::numeric_limits<std::size_t>::max();
    int max_size=1;
    for(const auto& a:p.agents)max_size=std::max(max_size,a.size);
    struct Estimate {Cost hmax=0,hsum=0,guide=0;bool possible=true;};
    auto estimate=[&](const std::vector<int>& c){
        Estimate e;
        for(int i=0;i<N;++i){
            int h=d.get(i,c[i]);if(h<0){e.possible=false;return e;}
            e.hmax=std::max(e.hmax,Cost(h));e.hsum+=h;
            e.guide+=Cost(h)*(4+4LL*p.agents[i].size/max_size);
        }
        return e;
    };
    std::vector<std::vector<int>> roots;
    std::vector<int> root(N),digits(N,0);
    for(int i=0;i<N;++i){roots.push_back(initial(p,o,i));root[i]=roots.back().front();}
    const auto first=estimate(root);
    if(!first.possible){result.status="UNSOLVABLE";return;}
    st.lb=o.soc()?first.hsum:first.hmax;
    // High-level best-first guide is deliberately inadmissible. Only the
    // independent distances above are reported as a proved lower bound.
    // With both independent caps, retain time in the duplicate key and
    // minimize SOC for that time. With one cap, its cost dominates.
    const bool timed=o.max_horizon>=0&&o.max_soc>=0;
    auto key=[&](const std::vector<int>& c,int t){auto k=c;if(timed)k.push_back(t);return k;};
    auto label=[&](int t,Cost soc){
        return o.max_soc>=0?std::pair<Cost,Cost>{soc,t}:std::pair<Cost,Cost>{t,soc};
    };
    using Item=std::tuple<Cost,Cost,std::uint64_t,std::size_t>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;
    std::vector<std::unique_ptr<LazyNode>> nodes;
    std::unordered_map<std::vector<int>,std::size_t,VecHash> best;
    std::uint64_t serial=0,root_attempts=0;
    std::mt19937 rng(o.seed);
    auto schedule=[&](std::size_t id){
        const auto& n=*nodes[id];
        Cost penalty=Cost(std::min<std::uint64_t>(n.attempts,1000000))/2;
        open.emplace(n.heuristic+Cost(n.t)*2+penalty,n.heuristic,serial++,id);
    };
    auto push=[&](std::vector<int> c,int t,Cost soc,std::size_t parent){
        auto e=estimate(c);
        if(!e.possible||(o.max_horizon>=0&&Cost(t)+e.hmax>o.max_horizon)||
           (o.max_soc>=0&&soc+e.hsum>o.max_soc))return;
        auto k=key(c,t);auto it=best.find(k);
        if(it!=best.end()){
            const auto& old=*nodes[it->second];
            if(label(old.t,old.soc)<=label(t,soc))return;
            // The old label is dominated; its pending work is obsolete.
            nodes[it->second]->pending.clear();
            nodes[it->second]->moves.clear();
            nodes[it->second]->order.clear();
        }
        auto id=nodes.size();
        nodes.push_back(std::make_unique<LazyNode>(std::move(c),parent,t,soc,e.guide));
        best[std::move(k)]=id;schedule(id);++st.joint_generated;
    };
    // Stay can have exponentially many valid initial commitment choices.
    // Generate them lazily as agenda items, rather than before search.
    bool more_roots=true;
    open.emplace(first.guide,first.guide,serial++,sentinel);
    st.phase="lacam_search";
    while(!open.empty()){
        st.open=open.size();st.tick(true);
        auto item=open.top();open.pop();auto id=std::get<3>(item);
        if(id==sentinel){
            if(!more_roots)continue;
            push(root,0,0,sentinel);++root_attempts;
            int i=0;
            for(;i<N;++i){
                if(++digits[i]<int(roots[i].size())){root[i]=roots[i][digits[i]];break;}
                digits[i]=0;root[i]=roots[i][0];
            }
            more_roots=i<N;
            if(more_roots)open.emplace(first.guide+2+Cost(std::min<std::uint64_t>(root_attempts,1000000)),
                                       first.guide,serial++,sentinel);
            continue;
        }
        auto& n=*nodes[id];
        auto it=best.find(key(n.config,n.t));
        if(it==best.end()||it->second!=id)continue;
        if(std::all_of(n.config.begin(),n.config.end(),[](int s){return done(s);})){
            Paths paths(N);
            for(auto k=id;k!=sentinel;k=nodes[k]->parent)
                for(int i=0;i<N;++i)paths[i].push_back(nodes[k]->config[i]);
            for(auto& path:paths)std::reverse(path.begin(),path.end());
            verify(p,o,paths);
            st.ub=o.soc()?n.soc:n.t;
            result={st.lb==st.ub?"OPTIMAL":"FEASIBLE",std::move(paths),true};
            return;
        }
        if(o.max_horizon>=0&&n.t>=o.max_horizon)continue;
        if(n.t>=std::numeric_limits<int>::max()-1)throw std::runtime_error("LaCAM horizon exceeds integer range");
        Cost new_soc=n.soc;
        for(int s:n.config)new_soc+=!done(s);
        if(o.max_soc>=0&&new_soc>o.max_soc)continue;
        if(!n.prepared){lazy_prepare(n,p,o,d,st,rng);++st.joint_expanded;}
        if(n.pending.empty())continue;
        auto prefix=std::move(n.pending.front());n.pending.pop_front();
        ++n.attempts;++st.lazy_attempts;
        // Add one level to the constraint tree. Invalid assigned pairs
        // cannot be extended, so pruning these prefixes is exact.
        auto k=prefix.states.size();
        if(k<n.order.size()){
            int i=n.order[k];
            for(int s:n.moves[i]){
                bool ok=true;
                for(std::size_t j=0;j<k;++j){
                    st.tick();
                    if(lazy_conflict(p,n.config,i,s,n.order[j],prefix.states[j])){ok=false;break;}
                }
                if(ok){auto child=prefix;child.states.push_back(s);n.pending.push_back(std::move(child));}
            }
        }
        auto successor=lazy_generate(p,n,prefix,st);
        if(!n.pending.empty())schedule(id);
        else {n.moves.clear();n.order.clear();}
        if(successor)push(std::move(*successor),n.t+1,new_soc,id);
        else ++st.lazy_failed;
    }
    result.status=bounded_failure(o);
}
}
