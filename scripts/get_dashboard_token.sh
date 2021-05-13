#!/bin/bash

k3s kubectl -n kubernetes-dashboard describe secret admin-user-token | grep ^token | cut -c 13-
