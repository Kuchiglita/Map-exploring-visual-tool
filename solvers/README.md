# Exact LA-MAPF solvers

## Discrete model (agreed 26.11.2026)

A state uses an integer center (x,y), an odd square side size, and optionally heading. All bodies must be contained in free cells and disjoint at every integer tick. A joint transition is synchronous. The **only** additional edge conflict is swapping the centers of two 1×1 agents. Swept volumes/linear interpolation are deliberately ignored, including perpendicular moves that overlap between ticks. PM: N/E/S/W/wait. RM: forward/left/right/wait; discrete turns leave the footprint unchanged. Initial RM heading N unless specified; goal heading unrestricted. All agents stay physically present.

Visit: first arrival completes the task, but the agent may continue moving. Stay: the agent may visit its goal and leave before its final arrival; after committing to completion it waits at the goal. All tasks must be completed by the final tick. For Stay the solver can defer commitment to a later goal visit. Completion is part of each search state.

Costs: makespan = maximum completion time, SOC = sum of completion times (first visit for Visit, terminal arrival for Stay). A solution ends when all tasks are completed. Movement after completion does not contribute to SOC, but collisions remain enforced. These objectives are finite-task proxies, not an optimal lifelong-throughput objective.

## Build and run

Requires CMake 3.16+, a C++17 compiler, Python 3.10+ (wrapper only). Same commands on Windows and Ubuntu:

    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build --config Release
    ctest --test-dir build -C Release --output-on-failure

    python solvers/run.py maps/elementary/demo-agents.json --solver mc-cbs --model PM --policy visit --objective makespan --max-horizon 30 --timeout-sec 120 --out runs/demo.json --viewer runs/demo.html

The wrapper accepts the existing v1 JSON scenarios with grid or map reference. Every agent must have a goal. Optional heading: N/E/S/W or 0/1/2/3 (PM ignores it). --exe overrides automatic executable lookup; --build-dir and --maps-dir override defaults. The binary reads a simple text protocol on stdin; no third-party C++ JSON library. Run --help for options. No platform-specific code.

## Algorithms

- joint-bfs: exact joint state enumeration. Priority is makespan (BFS layers, SOC tie-break), or Dijkstra by SOC. The augmented state includes every agent's completion bit. With an explicit horizon or SOC cap, time is also in the key; otherwise the best lexicographic label per configuration dominates worse ones. Exhaustion without bounds proves UNSOLVABLE. Intended only for small problems.
- cbs: fixed-horizon CBS, exact space-time A* per agent. Vertex split forbids the two individual conflicting centers, edge split forbids each individual swap transition.
- mc-cbs: same search, asymmetric multi-constraint vertex split (MC-CBS ASYM adapted to our discrete collision semantics). One branch forbids A's exact center; the other forbids all B centers overlapping that A center. Both include all headings. Edge split as CBS. This disjunction preserves completeness on a finite horizon; it is not a reproduction of every optimization in the paper.
- Low-level A* minimizes completion time subject to constraints and extends the path to the common horizon, allowing Visit agents to move after completion. Reverse individual BFS gives a lower bound; RM includes heading. CT best-first by sum of individual completion times. Constraint sets deduplicated.
- Makespan: try horizons from the individual maximum lower bound upward; the first feasible horizon is optimal. SOC: if a horizon/cost cap is specified, solve the fixed relevant horizon exactly. Otherwise first obtain an incumbent U, then solve at H=U (or a smaller proved finite state bound), since any better solution has makespan <= SOC < U. A finite state upper bound on the shortest solution is the product of individual augmented state counts minus one; computed only when representable. It is usually enormous. Without a usable bound CBS can time out on unsolvable instances.

## Bounds and reporting

--max-horizon N and --max-soc N are optional (-1 means no explicit bound). --timeout-sec defaults to 300 (0 disables). --report-interval-sec defaults to 60 (0 disables periodic reports, not the final report). A timeout never implies infeasibility. OPTIMAL and lower/upper bounds refer to the specified constraints; a capped-horizon SOC optimum need not be globally optimal without that cap. If no upper bound is known, it is null. Progress is JSON Lines on stderr and in <out>.progress.jsonl, including elapsed time, horizon, lower/upper bounds, CT generated/expanded, individual A* calls/expanded, joint generated/expanded and OPEN (outer frontier, approximate during child generation). Percent completion/ETA is not reported because search size is unknown. Lower bound is conservative; for global SOC before the proof phase it stays at the independent-agent bound. If all shorter makespan horizons fail, the next horizon is the proved lower bound. Verification and output serialization occur after search timeout checks; the timeout is not a hard process kill.

Statuses: OPTIMAL, UNSOLVABLE (global proof), INFEASIBLE_WITHIN_HORIZON, INFEASIBLE_WITHIN_COST, INFEASIBLE_WITHIN_BOUNDS (both caps), TIMEOUT_WITH_SOLUTION, TIMEOUT_NO_SOLUTION, RESOURCE_LIMIT_WITH_SOLUTION, RESOURCE_LIMIT_NO_SOLUTION. A validated incumbent, if any, is retained at timeout during the global SOC proof phase. Bad input/errors exit nonzero.

## Results and replay

The result JSON stores model, policy, objective, limits, status, bounds, statistics, makespan/SOC and paths, plus scenario geometry/agents. paths[i][t] = [x,y,heading,completed] in agent list order. A null paths value means no solution was produced. The binary verifies all paths and costs before returning. For a standalone browser replay:

    python solvers/replay.py runs/demo.json --out runs/demo.html

This is separate from the editor and requires no server/internet. Timeline, play/pause, step, zoom/pan and selected-agent path; size colors match Map Explorer. Only discrete frames are drawn.

## Current limitations

This is a correctness-oriented baseline, not an optimized CBS implementation: no conflict prioritization, disjoint splitting, cardinal heuristics, mutex propagation, incremental A*, bounded-suboptimal search or sophisticated memory management. Joint enumeration is exponential; repeated horizon searches can be expensive. Tests compare small instances against the joint solver and separately validate returned trajectories. No claim of SOTA or empirical throughput until measured.
