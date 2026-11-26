"""Independent trajectory checks and exact-solver cross comparisons."""
import itertools
from pathlib import Path
import tempfile
import json
import subprocess
import sys
import random
import unittest

EXE = sys.argv.pop(1) if __name__ == "__main__" and len(sys.argv) > 1 else None

def solve(grid, agents, solver="mc-cbs", model="PM", policy="visit", objective="makespan", H=None, S=None):
    text = f"{len(grid[0])} {len(grid)} {len(agents)}\n" + "\n".join(grid) + "\n"
    text += "".join(" ".join(map(str, a)) + "\n" for a in agents)
    cmd = [EXE, "--solver", solver, "--model", model, "--policy", policy,
           "--objective", objective, "--timeout-sec", "15", "--report-interval-sec", "0"]
    if H is not None:
        cmd += ["--max-horizon", str(H)]
    if S is not None:
        cmd += ["--max-soc", str(S)]
    run = subprocess.run(cmd, input=text, capture_output=True, text=True, timeout=20, check=True)
    result = json.loads(run.stdout)
    if result["status"] not in ("OPTIMAL", "UNSOLVABLE", "INFEASIBLE_WITHIN_HORIZON",
                               "INFEASIBLE_WITHIN_COST", "INFEASIBLE_WITHIN_BOUNDS"):
        raise AssertionError(result)
    if result["paths"] is not None:
        verify(grid, agents, result, model, policy)
    return result

def verify(grid, agents, result, model, policy):
    paths = result["paths"]
    assert len(paths) == len(agents)
    if not paths:
        assert result["makespan"] == result["soc"] == 0
        return
    T = len(paths[0])-1
    arrivals = []
    for a,p in zip(agents,paths):
        s,sx,sy,gx,gy,heading = a
        assert len(p)==T+1 and p[0][:2]==[sx,sy]
        assert p[0][2]==(heading if model=="RM" else 0)
        assert p[-1][3]
        tau = next(t for t,q in enumerate(p) if q[3])
        arrivals.append(tau)
        assert p[tau][:2]==[gx,gy]
        if policy=="visit":
            assert tau==next(t for t,q in enumerate(p) if q[:2]==[gx,gy])
        for t,(x,y,d,b) in enumerate(p):
            k=s//2
            assert k<=x<len(grid[0])-k and k<=y<len(grid)-k
            assert all(grid[j][i]=="." for j in range(y-k,y+k+1) for i in range(x-k,x+k+1))
            if t:
                px,py,pd,pb=p[t-1]
                assert not pb or b
                if policy=="stay" and pb:
                    assert [x,y,d]==[px,py,pd]
                if model=="PM":
                    assert abs(x-px)+abs(y-py)<=1
                elif (x,y)==(px,py):
                    assert (d-pd)%4 in (0,1,3)
                else:
                    dx,dy=((0,-1),(1,0),(0,1),(-1,0))[pd]
                    assert (x,y,d)==(px+dx,py+dy,pd)
    for t in range(T+1):
        for i,j in itertools.combinations(range(len(paths)),2):
            a,b=paths[i][t],paths[j][t]
            si,sj=agents[i][0],agents[j][0]
            assert 2*abs(a[0]-b[0])>=si+sj or 2*abs(a[1]-b[1])>=si+sj
            if t and si==sj==1:
                assert not (paths[i][t-1][:2]==b[:2] and paths[j][t-1][:2]==a[:2])
    assert max(arrivals)==result["makespan"]
    assert sum(arrivals)==result["soc"]
    assert result["lower_bound"]==result["upper_bound"]
    assert result["upper_bound"]==result[result["objective"]]

@unittest.skipUnless(EXE, "run with solver executable or through CTest")
class ExactTests(unittest.TestCase):
    def compare(self,grid,agents,model="PM",policy="visit",objective="makespan",H=5,S=None,expected=None):
        results=[solve(grid,agents,k,model,policy,objective,H,S) for k in ("joint-bfs","cbs","mc-cbs")]
        feasible=[r["status"]=="OPTIMAL" for r in results]
        self.assertTrue(all(x==feasible[0] for x in feasible),results)
        if feasible[0]:
            self.assertEqual(len({r[objective] for r in results}),1,results)
        if expected is not None:
            self.assertTrue(feasible[0],results)
            self.assertEqual(results[0][objective],expected)
        return results[0]

    def test_visit_vs_stay(self):
        # A begins on its goal and must move out of the way.
        agents=[(1,2,0,2,0,1),(1,0,0,3,0,1)]
        self.compare(["....."],agents,H=4,expected=3)
        r=self.compare(["....."],agents,policy="stay",H=4)
        self.assertNotEqual(r["status"],"OPTIMAL")

    def test_forbidden_swap(self):
        agents=[(1,0,0,1,0,1),(1,1,0,0,0,3)]
        for solver in ("joint-bfs","cbs","mc-cbs"):
            r=solve([".."],agents,solver,H=4)
            self.assertNotEqual(r["status"],"OPTIMAL")
        self.assertEqual(solve([".."],agents,"joint-bfs")["status"],"UNSOLVABLE")

    def test_cycle(self):
        a=[(1,0,0,1,0,1),(1,1,0,1,1,2),(1,1,1,0,1,3),(1,0,1,0,0,0)]
        self.compare(["..",".."],a,H=1,expected=1)
        self.compare(["..",".."],a,model="RM",H=1,expected=1)

    def test_large_follow_and_corner(self):
        a=[(5,2,2,3,2,1),(5,7,2,8,2,1)]
        self.compare(["."*11]*5,a,H=1,expected=1)
        # Both endpoints disjoint; continuous interpolation overlaps.
        a=[(5,2,2,3,2,1),(5,7,6,7,7,2)]
        self.compare(["."*10]*10,a,H=1,expected=1)

    def test_packed_large(self):
        a=[(3,1,1,4,1,1),(3,4,1,1,1,3)]
        self.assertEqual(solve(["......"]*3,a,"joint-bfs")["status"],"UNSOLVABLE")
        self.assertNotEqual(self.compare(["......"]*3,a,H=3)["status"],"OPTIMAL")

    def test_rotation_and_soc(self):
        a=[(1,0,0,2,0,3)]
        self.compare(["..."],a,model="RM",H=4,expected=4)
        self.compare(["..."],a,model="RM",H=4,objective="soc",expected=4)
        self.assertNotEqual(self.compare(["..."],a,model="RM",H=4,S=3)["status"],"OPTIMAL")

    def test_mixed_small(self):
        a=[(3,1,1,3,1,1),(1,4,1,0,1,3)]
        for model in ("PM","RM"):
            for policy in ("stay","visit"):
                for obj in ("makespan","soc"):
                    self.compare(["....."]*4,a,model,policy,obj,H=4)

    def test_global_soc_and_unsolvable(self):
        a=[(1,0,0,2,0,1),(1,2,0,0,0,3)]
        for policy in ("stay","visit"):
            self.compare(["...","..."],a,policy=policy,objective="soc",H=None)
        for solver in ("cbs","mc-cbs","joint-bfs"):
            self.assertEqual(solve([".."],[(1,0,0,1,0,1),(1,1,0,0,0,3)],solver)["status"],"UNSOLVABLE")

    def test_wrapper_unicode_replay(self):
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)/"карты и решения"
            folder.mkdir()
            case=folder/"сценарий.json"
            out=folder/"результат.json"
            viewer=folder/"просмотр.html"
            case.write_text(json.dumps({"version":1,"grid":["..."],
                "agents":[{"id":7,"size":1,"start":[0,0],"goal":[2,0],"heading":"E"}]}),encoding="utf-8")
            subprocess.run([sys.executable,str(root/"solvers/run.py"),str(case),"--exe",EXE,
                "--model","RM","--max-horizon","2","--out",str(out),"--viewer",str(viewer)],
                check=True,capture_output=True,timeout=20,env={**__import__("os").environ,"PYTHONIOENCODING":"utf-8"})
            data=json.loads(out.read_text(encoding="utf-8"))
            self.assertEqual((data["status"],data["makespan"]),("OPTIMAL",2))
            self.assertEqual(data["agents"][0]["id"],7)
            self.assertTrue(out.with_suffix(".progress.jsonl").is_file())
            self.assertNotIn("/*__DATA__*/",viewer.read_text(encoding="utf-8"))

    def test_random_small(self):
        rng=random.Random(42)
        for _ in range(12):
            starts=rng.sample(range(6),2);goals=rng.sample(range(6),2)
            agents=[(1,s%3,s//3,g%3,g//3,rng.randrange(4)) for s,g in zip(starts,goals)]
            for obj in ("makespan","soc"):
                self.compare(["...","..."],agents,policy=rng.choice(["visit","stay"]),objective=obj,H=4)

if __name__=="__main__":
    if EXE is None:
        raise SystemExit("usage: python tests/test_solvers.py PATH_TO_LA_SOLVER")
    unittest.main()
