#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>

// split a string at any spaces, commas, endlines, EOF
void split_string(std::string stringInput, std::vector<std::string>& vector_of_strings) {
    std::string temp_string = "";
    for (int i = 0; i < stringInput.length(); i++) {
        if ((stringInput[i] != ' ') && (stringInput[i] != '\n') && (stringInput[i] != ',') && (stringInput[i] != '\0')) {
            temp_string.push_back(stringInput[i]);
        } 
        else {
            vector_of_strings.push_back(temp_string);
            if (stringInput[i] == '\0') {
                std::cout << "exiting" << "\n";
                return;
            }
            else {
                temp_string = "";
            }
        }
    }
    vector_of_strings.push_back(temp_string);
}

int main(int argc, char **argv) {
    std::ifstream inf("goals.txt");
    double goals[100][2];
    int goal_ind = 0;
    
    while (inf) {
        std::string stringInput;
        std::vector<std::string> vector_of_strings;
        std::getline(inf, stringInput);
        // check if this is the EOF
        if (stringInput[0] == '\0') {
            break;
        }
        split_string(stringInput, vector_of_strings);
        //std::cout << atof(vector_of_strings[0].c_str()) << ", " << atof(vector_of_strings[1].c_str()) << "\n";
        //std::cout << "count: " << count << "\n";
        goals[goal_ind][0] = atof(vector_of_strings[0].c_str());
        goals[goal_ind][1] = atof(vector_of_strings[1].c_str());
        //std::cout << goals[goal_ind][0] << ", " << goals[goal_ind][1] << "\n";
        goal_ind++;
        //std::cout << goals[goal_ind][0] << ", " << goals[goal_ind][1] << "\n";
    }
}

