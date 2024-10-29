

# Start to get data using loop wait

```
roslaunch reconstruction sensor_start.launch 

roslaunch reconstruction sync_get_data_loop.launch
```

After these code, first you need to enter 'y', then you need to enter a second period of time, which is the time you want to wait for the data to be collected.

# Start to get data using gimbal

```
roslaunch reconstruction sensor_start.launch 

roslaunch reconstruction sync_get_data_with_gimbal.launch 
```
After these code, you need to enter 'y' to get the data.





