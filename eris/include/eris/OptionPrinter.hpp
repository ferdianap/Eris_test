/**********************************************************************************************************************
**         __________              ___                              ________                                         **
**         \______   \_____     __| _/ _____  _____     ____       /  _____/ _____     _____    ____    ______       **
**          |       _/\__  \   / __ | /     \ \__  \   /    \     /   \  ___ \__  \   /     \ _/ __ \  /  ___/       **
**          |    |   \ / __ \_/ /_/ ||  Y Y  \ / __ \_|   |  \    \    \_\  \ / __ \_|  Y Y  \\  ___/  \___ \        **
**          |____|_  /(____  /\____ ||__|_|  /(____  /|___|  /     \______  /(____  /|__|_|  / \___  \/____  \       **
**                 \/      \/      \/      \/      \/      \/             \/      \/       \/      \/      \/        **
**                                                         2012                                                      **
**********************************************************************************************************************/

#ifndef RAD_PRETTYOPTIONPRINTER_HPP
#define RAD_PRETTYOPTIONPRINTER_HPP

#include <eris/CustomOptionDescription.hpp>

class OptionPrinter {
public:
    void addOption(const CustomOptionDescription& optionDesc);

    std::string usage();

    std::string positionalOptionDetails();
    std::string optionDetails();

public:
    static void printStandardAppDesc(const std::string& appName, std::ostream& out,
                                     boost::program_options::options_description desc,
                                     boost::program_options::positional_options_description* positionalDesc=0);
    static void formatRequiredOptionError(boost::program_options::required_option& error);

private:
    std::vector<CustomOptionDescription> options;
    std::vector<CustomOptionDescription> positionalOptions;

};

#endif // RAD_PRETTYOPTIONPRINTER_HPP