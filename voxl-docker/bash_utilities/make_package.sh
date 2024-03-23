#!/bin/bash
################################################################################
# Copyright (c) 2022 ModalAI, Inc. All rights reserved.
#
# author: alex.gardner@modalai.com
################################################################################

./clean.sh
cd fs
tar -cvf bash_utilities.tar * > /dev/null
mv bash_utilities.tar ../
