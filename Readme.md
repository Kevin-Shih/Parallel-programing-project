# 2024-Fall NYCU Parallel Programing Final Project

## Parallelization of RRT Algorithm
We Parallelizd widely used path finding algorithm RRT using OpenMP (and Pthread, on feature branch) and achieve reasonable speedup.

## Usage
1.  Install by running the `install.sh` script
2.  Run Parallel RRT by `./RRT -m 0 -v -p`. (By Default 8 threads)
    Run Serial by `./RRT_serial -m 0 -v -p`. 
3.  All the command line option listed here. Use `-h`, `--help` to show this message
    ```
    Usage: RRT [options]
    Program Options:
      -i  --iter    <INT>   Test iterations(>1)
      -m  --map     <INT>   Input map (0, 1, 2, 3)
      -r  --radius  <FLOAT> Radius to inflate the obstacles
      -l  --steplen <FLOAT> Step length for getting new nodes(>15)
      -s  --std     <FLOAT> Std for generate rand node
      -p  --plot            Whether to plot the result and save
      -v  --verbose <INT>   Whether to print info
      -h  --help            This message
    ```