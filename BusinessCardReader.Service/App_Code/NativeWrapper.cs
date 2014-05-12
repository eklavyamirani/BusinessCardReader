using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Web;

namespace BusinessCardReader.Service
{
    public class NativeWrapper
    {
        [StructLayout(LayoutKind.Sequential)]
        protected struct _contactInformation
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256, ArraySubType = UnmanagedType.I1)]
            public byte[] Name;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256, ArraySubType = UnmanagedType.I1)]
            public byte[] Phone;

            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256, ArraySubType = UnmanagedType.I1)]
            public byte[] Email;

        }


        [DllImport(@"TextDetection.dll",
            CallingConvention=CallingConvention.StdCall,
            EntryPoint = "DetectTextInImage",
            CharSet=CharSet.Ansi)]
        protected static extern int textDetection(IntPtr imageData, int size, ref _contactInformation contactInformation);

        public static BusinessCardReader.Service.Models.ContactInformation DetectText(byte[] imageData)
        {
            if(imageData == null)
            {
                return null;
            }
            IntPtr imageDataReference = IntPtr.Zero;
            imageDataReference = Marshal.AllocHGlobal(imageData.Length + 1);
            Marshal.Copy(imageData, 0, imageDataReference, imageData.Length);
            Marshal.WriteByte(imageDataReference + imageData.Length, 0);

            var contactInformation = new _contactInformation();

            try
            {
                textDetection(imageDataReference, imageData.Length + 1, ref contactInformation);
                var contactDetailsConverted = new Models.ContactInformation();
                
                contactDetailsConverted.Name = Encoding.ASCII.GetString(contactInformation.Name
                    .TakeWhile<byte>(value => value != 0)
                    .ToArray<byte>());

                contactDetailsConverted.Email = Encoding.ASCII.GetString(contactInformation.Email
                    .TakeWhile<byte>(value => value != 0)
                    .ToArray<byte>());

                contactDetailsConverted.Phone = Encoding.ASCII.GetString(contactInformation.Phone
                    .TakeWhile<byte>(value => value != 0)
                    .ToArray<byte>());

                return contactDetailsConverted;
            }
            catch(Exception e)
            {
                return null;
            }
            finally
            {
                if (imageDataReference != IntPtr.Zero)
                {
                    Marshal.FreeHGlobal(imageDataReference);
                }
            }
        }
    }
}