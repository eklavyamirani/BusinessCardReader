#include <iostream>
#include <regex>
#include "PatternMatching.h"

namespace BusinessCardReader
{
	namespace PatternMatching
	{
		std::string CommonString(std::string name, std::string email)
		{
			std::string common, max;

			for (int i = 0; i < name.length(); i++)
			for (int j = 0; j < email.length(); j++)
			for (int k = 1; k <= name.length() && k <= email.length(); k++){
				if (name.substr(i, k) == email.substr(j, k)){
					common = name.substr(i, k);
				}
				else{
					if (common.length() > max.length())
						max = common;
					common = "";
				}
			}
			if (common.length() > max.length())
				max = common;
			common = "";

			return max;
		}


		std::string MatchEmailToName(std::smatch matches, const std::string email)
		{
			for each(std::string result in matches)
			{
				std::string com = CommonString(result, email);
				if (com.size() > 2)
					return result;
			}
		}

		std::string MatchName(std::vector<std::string> inputTextCollection, const std::string email)
		{
			std::regex nameRegex("^(([a-z]|[A-Z])(([a-z]|[A-Z])*|\\.) *){1,2}([a-z][a-z]+-?)+$");
			std::smatch matchedStrings;
			for each (std::string inputText in inputTextCollection)
			{
				if (std::regex_search(inputText, matchedStrings, nameRegex))
				{
					return MatchEmailToName(matchedStrings, email);
				}
				
			}
			return "";
		}

		std::string MatchEmail(const std::vector<std::string> inputTextCollection)
		{
			std::regex emailRegex("([^ \n]+ *@ *.+(\\..{2,4})+)$");
			std::smatch matchedString;
			for each (std::string inputText in inputTextCollection)
			{
				if (std::regex_search(inputText, matchedString, emailRegex))
				{
					return matchedString[0];
				}
			}
			return "";
		}

		std::string MatchPhone(const std::vector<std::string> inputTextCollection)
		{
			std::regex phoneRegex("(?:^|\\D)(\\d{3})[)\\-. ]*?(\\d{3})[\\-. ]*?(\\d{4})(?:$|\\D)");
			std::smatch matchedString;
			for each (std::string inputText in inputTextCollection)
			{
				if (std::regex_search(inputText, matchedString, phoneRegex))
				{
					return matchedString[0];
				}
			}
			return "";
		}

		ContactInformation ExtractContactInformation(std::vector<std::string> inputTextCollection)
		{
			ContactInformation contact;
			contact.Email = MatchEmail(inputTextCollection);
			contact.Name = MatchName(inputTextCollection, contact.Email);
			contact.Phone = MatchPhone(inputTextCollection);
			return contact;
		}
	}
}