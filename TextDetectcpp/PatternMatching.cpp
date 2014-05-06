#include <iostream>
#include <regex>
#include "PatternMatching.h"

namespace BusinessCardReader
{
	namespace PatternMatching
	{
		std::string MatchEmailToName(std::smatch matches, std::string email)
		{
#pragma message("Not Implemented")
		}

		std::string MatchName(std::vector<std::string> inputTextCollection)
		{
			std::regex nameRegex("^(([a-z]|[A-Z])(([a-z]|[A-Z])*|\\.) *){1,2}([a-z][a-z]+-?)+$");
			std::smatch matchedStrings;
			for each (std::string inputText in inputTextCollection)
			{
				if (std::regex_search(inputText, matchedStrings, nameRegex))
				{
					return matchedStrings[0];
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
			contact.Name = MatchName(inputTextCollection);
			contact.Email = MatchEmail(inputTextCollection);
			contact.Phone = MatchPhone(inputTextCollection);
			return contact;
		}
	}
}