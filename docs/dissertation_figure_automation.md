# Dissertation Figure Automation

This workspace now includes portable scripts to generate figure/source assets for section `5.4`:

- `scripts/dissertation/export_static_assets.py`
- `scripts/dissertation/capture_runtime_assets.sh`
- `scripts/dissertation/run_all.sh`

All generated files are written under:

- `docs/dissertation_assets/static/`
- `docs/dissertation_assets/runtime/<timestamp>/`

## 1) One-command run (recommended)

Run this while your ROS stack is up (and bag replay active if you want replay evidence):

```bash
cd ~/ROS_Deployment
chmod +x scripts/dissertation/*.sh
scripts/dissertation/run_all.sh
```

## 2) What gets generated

### Static assets (no running ROS required)

- `workspace_tree.txt` -> workspace/package structure figure source
- `package_dependency_graph.dot` -> package/module dependency graph source
- `package_dependency_graph.png/.pdf` -> rendered dependency figure (if `dot` installed)
- `launch_config_merge.mmd` -> launch/config layering diagram source (Mermaid)

### Runtime assets (ROS graph must be live)

- `node_list.txt` -> active nodes
- `topic_list_types.txt` -> topic/type table seed
- `topic_info_*.txt` -> QoS/endpoints for communication architecture
- `frames_*.gv/.pdf` -> TF tree figure source/output
- `node_info_*.txt` + `params_*.yaml` -> node architecture and config evidence
- `hz_*.txt` -> topic-rate evidence for pipeline timing/rates

## 3) Transfer to another computer

Copy only the automation scripts + docs folder:

```bash
rsync -av ~/ROS_Deployment/scripts/dissertation ~/ROS_Deployment/docs/dissertation_figure_automation.md <user>@<host>:~/ROS_Deployment/scripts/
```

Then on the other machine:

```bash
cd ~/ROS_Deployment
chmod +x scripts/dissertation/*.sh
scripts/dissertation/run_all.sh
```

## 4) Optional dependencies

- Graph rendering: `sudo apt install graphviz`
- Mermaid render (optional, if you want PNG/SVG from `.mmd`): use Mermaid CLI (`mmdc`) or VS Code Mermaid extension.

## 5) Suggested mapping to dissertation sections

- `5.4.1`: `package_dependency_graph.*` + high-level block diagram (draw.io/Figma from the same modules)
- `5.4.2`: `topic_list_types.txt` + `hz_*.txt`
- `5.4.4`/`5.4.5`: `node_info_*.txt` + `topic_info_*.txt`
- `5.4.6`: `frames_*.pdf`
- `5.4.7`: `workspace_tree.txt`
- `5.4.8`: `launch_config_merge.mmd`
