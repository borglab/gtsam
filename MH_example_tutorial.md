MH-iSAM2 Example Tutorial
=========================

To run an example that uses MH-iSAM2, type one of the following commands in the build/ directory after installing MH-iSAM2:
```
#!bash
$ ./examples/MH_ISAM2_TEST_city10000
```
or
```
#!bash
$ ./examples/MH_ISAM2_TEST_victoriaPark
```

Each of them will read its corresponding input data file from data/ directory. The data files are modified from city10000 or Victoria Park though adding ambiguous measurements, data associations, or loop closures. 

The format of the modified dataset with ambiguities is defined in [data/MH_data_format.txt](https://bitbucket.org/rpl_cmu/mh-isam2_lib/src/master/data/MH_data_format.txt).

Output files are created in the build/ directory after running each example:
```
*_hypos.txt
```
and
```
*_time.txt
```

The resulting trajectoy/map can be visualized using the following Matlab viewers with corresponding dataset names:

```
MH_plot_city10000.m
MH_plot_victoriaPark.m
```

The results should be similar to the left figures in Fig. 8-a or 8-b in [MH-iSAM2]()


To play with the examples, simply change the following parameters:

Length of the sequence:
```
max_loop_count

max_odom_count
```

Different types and combinations of input ambiguities:
```
input_file_name
```

To treat every loop closure as an ambiguous measurement or not
```
active_all_loop_detach
```

