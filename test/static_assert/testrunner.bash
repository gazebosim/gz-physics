#!/bin/bash

# mess with output to confuse jenkins build failure indications because these
# tests pass when compilation fails. Multiple pipes for OSX compatibility
eval $@ 2>&1 \
  | sed s/[Ee]rror/[Censored\ by\ testrunner.bash]/g \
  | sed s/Fail/[Censored\ by\ testrunner.bash]/g
