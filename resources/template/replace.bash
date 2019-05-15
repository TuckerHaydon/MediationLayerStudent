#!/bin/bash
cp obstacle.template obstacle.tmp
while read n k; do 
  sed -i "s/$n/$k/g" obstacle.tmp
done < $1
mv obstacle.tmp $2
