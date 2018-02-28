import os
import re
import yaml
from collections import OrderedDict

if __name__ == '__main__':
    filename="/homeL/demo/ws_grasp/src/upstream/gqcnn/cfg/ros_nodes/grasp_planner_node.yaml"
    fh = open(filename, 'r')
    file_contents = fh.read()
    # Replace !include directives with content
    config_dir = os.path.split(filename)[0]
    include_re = re.compile('^!include\s+(.*)$', re.MULTILINE)
    def include_repl(matchobj):
        fname = os.path.join(config_dir, matchobj.group(1))
        with open(fname) as f:
            return f.read()
    while re.search(include_re, file_contents): # for recursive !include
        file_contents = re.sub(include_re, include_repl, file_contents)

    # Read in dictionary
    class OrderedLoader(yaml.Loader):
        pass
    OrderedLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
        lambda loader, node: OrderedDict(loader.construct_pairs(node)))
    config=yaml.load(file_contents, OrderedLoader)
    # Convert functions of other params to true expressions
    for k in config.keys():
        if type(config[k]) is str and len(config[k]) > 2 and config[k][1] == '!':
            config[k] = eval(config[k][2:-1])
    fh.close()
    policy_cfg=config['policy']
    print policy_cfg
