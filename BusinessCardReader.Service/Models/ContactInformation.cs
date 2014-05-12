using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Web;

namespace BusinessCardReader.Service.Models
{
    public class ContactInformation
    {
        public string Name;
        public string Phone;
        public string Email;

        public string FirstName
        {
            get
            {
                return Name.Split(' ')[0];
            }
        }

        public string LastName
        {
            get
            {
                var names = Name.Split(' ');
                return names[names.Count()-1];
            }
        }

        public string CreateVCFCard()
        {
            var cardBuilder = new StringBuilder();
            cardBuilder.AppendLine("BEGIN:VCARD");
            cardBuilder.AppendLine("VERSION:2.1");
            cardBuilder.AppendLine("N:" + LastName + ";" + FirstName);
            cardBuilder.AppendLine("FN:" + Name);
            cardBuilder.AppendLine("TEL;WORK;VOICE:" + Phone);
            cardBuilder.AppendLine("EMAIL;PREF;INTERNET:" + Email);
            cardBuilder.AppendLine("END:VCARD");
            return cardBuilder.ToString();
        }
    }
}