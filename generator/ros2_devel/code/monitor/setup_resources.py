# so that we can get the the scripts path
# so that we can get the the scripts path
import os


# helper function for getting the files 
def get_files(root_path,ext,prefix):
    all_files = []
    for root, dirs, files in os.walk(root_path):
        for filename in files:
            if filename.lower().endswith(ext) and filename.lower().startswith(prefix):
                if filename != '__init__.py':
                    all_files.append( filename )
    return all_files

def script_name(sf):
    sfname = sf.replace("default_","")
    sfname = sfname.replace("default","")
    sfname = sfname.replace(".py","")
    return sfname

def create_entrypoint_line(sf,package_name):
    sfname = script_name(sf)
    line = sfname+' = '+package_name+'.'+sf.replace(".py",":main")
    return line

def console_script_lines(scriptfiles,package_name):
    sflines = []
    for sf in scriptfiles:
        line = create_entrypoint_line(sf,package_name)
        sflines.append(line)
    return sflines



def get_console_script_lines(package_name):
    script_path = os.path.dirname(os.path.realpath(__file__))
    scriptfiles = get_files(script_path+'/'+package_name+"/" ,".py","")
    sflines = console_script_lines(scriptfiles,package_name)
    return sflines