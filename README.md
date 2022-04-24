# GraphSLAM
An implementation of graph-based SLAM on a 2D indoor dataset. Course project of ECE1505, Convex Optimization.

Inspiration has been gained from Chenge Yang's repo *SLAM Algorithm Implementation - Probabilistic Robotics*, the third part Graph_SLAM, link to it: https://github.com/ChengeYang/Probabilistic-Robotics-Algorithms

## Dataset  
UTIAS Multi-Robot Cooperative Localization and Mapping Dataset  
Link: http://asrl.utias.utoronto.ca/datasets/mrclam/index.html
In this implementation, only a subset of the original dataset is used.

## Dependencies
To install the required denpendencies, simply run
```python 
pip install -r requirements.txt
```

## Execution
To use the default settings, simply run   
```python 
python main.py
```
Or, specify your own settings and run  
```python 
python main.py --meas "path_to_your_measurements" --init "path_to_your_initial_guess"`--max_iter your_maximum_iteration --tol your_tolerance --output_txt "directory_to_place_the_output_txt_files_recording_the_node_info" --output_plot "directory_to_place_the_output_plots"
```
