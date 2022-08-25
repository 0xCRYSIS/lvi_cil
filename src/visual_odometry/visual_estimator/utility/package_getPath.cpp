#include "package_getPath.h"

std::string get_package_path(std::string package_name)
{
    std::string share_dir = ament_index_cpp::get_package_share_directory(package_name);
    std::string pkg_path;
    int COUNT = 0;

    for(char i : share_dir)
    {   
        pkg_path += i;

        if(i == '/')
        {
            COUNT += 1;
        }

        if(COUNT == 4)
        {
            break;
        }

    }

    return pkg_path + "src/" + package_name;
}