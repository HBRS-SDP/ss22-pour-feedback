import roslaunch
package = 'hsr_vision'
executable = 'vision.py'
node_name = 'HSR_Vision'
node = roslaunch.core.Node(package=package, node_type=executable, name=node_name)
launch = roslaunch.scriptapi.ROSLaunch()
launch.start() 
process = launch.launch(node)
