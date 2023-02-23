/**
 * @file
 * @brief Header file consists declaration and definition class to parser command line arguments.
 * 
 * Link: https://stackoverflow.com/a/868894
*/ 

#ifndef ROBOTINO4_LIB__INPUTPARSER_HPP_
#define ROBOTINO4_LIB__INPUTPARSER_HPP_ 

#include <string>
#include <algorithm>

/**
 * @brief Class to parse command line arguments.
*/
class InputParser{
    public:

        /**
         * @brief Class constructor.
         * 
         * @param argc number of command line arguments. It corresponds to the argc argument of main function.
         * @param argv array with command line arguments. It corresponds to the argv argument of main function.
        */
        InputParser (int argc, char **argv){
            for (int i=1; i < argc; ++i)
                this->tokens.push_back(std::string(argv[i]));
        }

        /**
         * @brief Function to get value of non-position command line argument.
         * 
         * @param option command line argument name. Eg. -o, --name, etc.
         * @return string with value of command line argument.
         * 
         * Example:
         * @code
         * std::string ip_addr = input.getCmdOption("--ip");
         * 
         * unsigned int test_duration = std::stoi(input.getCmdOption("-t"));
         * @endcode
        */
        const std::string& getCmdOption(const std::string &option) const{
            std::vector<std::string>::const_iterator itr;
            itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
            if (itr != this->tokens.end() && ++itr != this->tokens.end()){
                return *itr;
            }
            static const std::string empty_string("");
            return empty_string;
        }
        
        /**
         * @brief Function to check non-position command line argument.
         * 
         * @param option command line argument name. Eg. -o, --name, etc.
         * @return true if argument passes, otherwise false.
         * 
         * Example:
         * @code
         * if(input.cmdOptionExists("--ip")) {
         *     ip_addr = input.getCmdOption("--ip");
         * }
         * if(input.cmdOptionExists("-t")) {
         *     test_duration = std::stoi(input.getCmdOption("-t"));
         * }
         * @endcode
        */
        bool cmdOptionExists(const std::string &option) const{
            return std::find(this->tokens.begin(), this->tokens.end(), option)
                   != this->tokens.end();
        }

    private:
        std::vector <std::string> tokens;
};

#endif  // ROBOTINO4_LIB__INPUTPARSER_HPP_
