**Trajectories and CPU/RAM usage from experiments described in the article can be found in this folder.**

To plot the trajectories obtained from the Newer College dataset superimposed to the localisation ground truth and their APE, use :
```bash
cd ~/olcmr/results/trajectories
python3 traj_plot_newer_college.py nc-quad-easy
```

To plot CPU/RAM evaluation on Newer College dataset, use :
```bash
cd ~/olcmr/results/cpu_usage
python3 cpu_eval.py cpu_eval_newer_college.txt
```

