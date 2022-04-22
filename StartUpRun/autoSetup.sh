#!/bin/bash

cp ./trisept.service /lib/systemd/system/
systemctl enable trisept.service
systemctl daemon-reload
