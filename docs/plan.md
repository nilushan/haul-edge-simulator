# Implementatin plan

Realistic Maps : Sensors should generate a few realistic maps. If data can be retrived from a online point cloud of a mine, that would be great. wonder if google map or a similar map can provide realistic 3d map data that can be then turned into a streamable point cloud. 

I want to run a few item detection ros nodes to be implemented. When a item is detected, want to fire a event with the item and all details like possition, time, what vehicle detected etc. Also want the event, data to be stored within the vehicle storage. And retrivable. Could be in a file or in a sqlite db. Purpose is to act like a edge node storage that will sync to a cloud storage when edge is online. 

