#!/bin/bash
cp obstacle.template obstacle.tmp
while read n k; do 
  sed -i "s/$n/$k/g" obstacle.tmp
done < replace.table
mv obstacle.tmp obstacle.yaml
