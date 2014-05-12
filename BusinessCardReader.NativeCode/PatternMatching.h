#ifndef PATTERN_MATCHING
#define PATTERN_MATCHING
#include <iostream>
#include <regex>

namespace BusinessCardReader
{
	struct ContactInformation
	{
		std::string Name;
		std::string Phone;
		std::string Email;
	};

	namespace PatternMatching
	{
		std::string MatchEmailToName(std::smatch matches, std::string email);
		
		std::string MatchEmail(std::vector<std::string> inputTextCollection);

		std::string MatchName(std::vector<std::string> inputTextCollection, const std::string email);

		std::string MatchPhone(std::vector<std::string> inputTextCollection);

		ContactInformation ExtractContactInformation(std::vector<std::string> inputTextCollection);
	}
}
#endif