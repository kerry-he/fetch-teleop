# Data
These scripts are used to process the ROS bag data files obtained. 

## Merge bags

There should be two bags: one from the Fetch and one from the local machine. To merge these bags together, run:

    python3 merge_bags.py

When prompted, input the prefix of the bags you want to merge. For example, to merge `fetch_test_2022-06-27-00-05-43.bag` and `fetch_test_2022-06-27-17-30-39.bag`, input the prefix `fetch_test`. This will produce a combined bag called `fetch_test_combined.bag`


## Bag to CSV

To write data from the bag file into  a CSV for training, run the following:

    roscore
    source fetch_ws/devel/setup.bash 
    python3 bag_to_csv.py

Similar to the merge bag script, you will be prompted to input the prefix of the bag you want to convert. For example, if you input the prefix `fetch_test`, then a bag of the form `fetch_test_<>_combined.bag` will be converted to CSV.

