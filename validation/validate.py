import random
import sys

import numpy as np
from scipy.linalg import expm

from tools import good, run, save

binary=sys.argv[1]
base={"mass_kg":1,"spring_n_per_m":1,"damping_ns_per_m":.4,"initial_x_m":1,"initial_v_m_s":.2,
      "target_m":0,"dt_s":.08,"horizon_s":2.05,"force_limit_n":3,"position_limit_m":2,
      "control":{"kind":"held","switches_s":[.65,1.23]}}
forces=[-.4,.3,.1]
def reference(model, forces):
    x=np.array([model["initial_x_m"],model["initial_v_m_s"],1.])
    times=[0,*model["control"]["switches_s"],model["horizon_s"]]
    for lo,hi,force in zip(times[:-1],times[1:],forces,strict=True):
        m=model["mass_kg"]
        A=np.array([[0,1,0],[-model["spring_n_per_m"]/m,-model["damping_ns_per_m"]/m,force/m],[0,0,0]])
        x=expm(A*(hi-lo))@x
    return x[:2]
results=[]
for damping in [.4,2,4]:
    model=base | {"damping_ns_per_m":damping}
    expected=reference(model,forces)
    errors=[]
    for dt in [.08,.04,.02]:
        row=good(run(binary,model | {"dt_s":dt},[forces],trace=True)[0])
        assert row["trace"][-1][0]==model["horizon_s"]
        for switch in model["control"]["switches_s"]:
            assert any(p[0]==switch for p in row["trace"])
        errors.append(float(max(abs(expected-[row["terminal_position_m"],row["terminal_velocity_m_s"]]))))
    assert errors[2] < errors[1] < errors[0] and errors[-1]<1e-6, errors
    results.append({"damping":damping,"errors":errors})
assert run(binary,base,[[10,0,0]])[0]["status"]=="invalid"
control=base | {"control":{"kind":"pd","interval_s":.1},"dt_s":.04}
candidates=[[p,d] for p in range(5) for d in range(4)]
def scenarios(seed):
    rng=random.Random(seed)
    return [control | {"mass_kg":rng.uniform(.8,1.2),"damping_ns_per_m":rng.uniform(.25,.65)} for _ in range(6)]
def compare(seed,dt):
    return list(zip(*(run(binary,m|{"dt_s":dt},candidates) for m in scenarios(seed)),strict=True))
train=compare(8,.04)
winner=min(range(len(candidates)),key=lambda i:sum(r["score"] for r in train[i]))
hold=compare(29,.01)
assert sum(r["score"] for r in hold[winner])<sum(r["score"] for r in hold[0])
nominal=run(binary,control,candidates)
archive={}
for i,r in enumerate(nominal):
    m=good(r)
    cell=(int(m["integrated_error"]*4),int(m["integrated_force_squared"]*2))
    if cell not in archive or r["score"]<nominal[archive[cell]]["score"]:
        archive[cell]=i
for i in archive.values():
    assert run(binary,control|{"dt_s":.01},[candidates[i]])[0]["status"]=="complete"
save({"model":control,"parameters":candidates[winner]}, {"reference":results,"winner":candidates[winner],
     "holdout_baseline":[r["score"] for r in hold[0]],"holdout_winner":[r["score"] for r in hold[winner]],
     "holdout_failed":0,"feasible_descriptor_cells":len(archive),"descriptor":"integrated error and effort bins"})
