#!/bin/bash

gen_dir=$1
#evaluation/parallel_simulation/gen

for expdir in `ls $gen_dir`; do
  find $gen_dir/$expdir -maxdepth 1 -type d -print0 | while read -d '' -r dir; 
    do num=$(find $dir -name sim_data.bag | wc -l); printf "%5d files in directory %s\n" "$num" "$dir"; 
  done; echo ''; 
done
