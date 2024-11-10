# MultiMapNavigation Readme

## Getting Started

Follow these instructions to set up and run the MultiMapNavigation system.

### 1. Clone the Repository

Download the MultiMapNavigation repository from GitHub:

```bash
git clone https://github.com/yourusername/MultiMapNavigation.git
cd MultiMapNavigation
```

### 2. Start the ANSCER System

Launch the ANSCER system:

```bash
roslaunch start_anscer start_anscer.launch
```

### 3. Start ANSCER Navigation

To launch the navigation system:

```bash
roslaunch anscer_navigation anscer_navigation.launch
```

**Note:**  
Before running this, edit the launch file to specify the default map of your choice.

### 4. Start Multimap Navigation Action Server

Run the multimap navigation action server:

```bash
rosrun multimap_navigation multimap_navigation_action_server
```

### 5. Run Multimap Navigation Client

Navigate to a specific location on a specified map using the client:

```bash
rosrun multimap_navigation multimap_navigation_client.py <map_name> <x> <y> <z>
```

### Important Notes

1. **Database Update:**  
   Ensure the `db` folder is updated with a database containing the map names and their corresponding coordinates.

2. **Maps Folder:**  
   The `map` folder must contain all the maps you intend to use for multimap navigation. Make sure each map is correctly formatted and accessible.

---
