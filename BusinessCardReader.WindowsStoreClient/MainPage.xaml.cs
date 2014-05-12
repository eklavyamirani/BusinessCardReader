using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Text;
using Windows.ApplicationModel.Contacts;
using Windows.ApplicationModel.Contacts.Provider;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.Media.Capture;
using Windows.Storage.Pickers;
using Windows.UI.Popups;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Media.Imaging;
using Windows.UI.Xaml.Navigation;

// The Blank Page item template is documented at http://go.microsoft.com/fwlink/?LinkId=234238

namespace BusinessCardReader.WindowsStoreClient
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        private Windows.Storage.StorageFile _file;
        private const string _serviceUri = "http://localhost:38259/api/values";
        public MainPage()
        {
            this.InitializeComponent();
        }

        private async void CaptureImageButton_Click(object sender, RoutedEventArgs e)
        {
            var ui = new CameraCaptureUI();
            _file = await ui.CaptureFileAsync(CameraCaptureUIMode.Photo);

            await SetImageInHolder(_file);
        }

        private async System.Threading.Tasks.Task SetImageInHolder(Windows.Storage.StorageFile file)
        {
            if (file != null)
            {
                var bitmap = new BitmapImage();
                await bitmap.SetSourceAsync(await file.OpenReadAsync());
                CardImageHolder.Source = bitmap;
            }
        }

        private async void SendCardButton_Click(object sender, RoutedEventArgs e)
        {
            var bitmapFileStream = await  _file.OpenAsync(Windows.Storage.FileAccessMode.Read);
            var convertedImage = new byte[bitmapFileStream.Size];
            await bitmapFileStream.ReadAsync(convertedImage.AsBuffer(), (uint)bitmapFileStream.Size, Windows.Storage.Streams.InputStreamOptions.None);
            var value = Newtonsoft.Json.JsonConvert.SerializeObject(convertedImage);
            System.Diagnostics.Debug.WriteLine(value.Length);
            byte[] dataToSend = Encoding.UTF8.GetBytes(value);
            var request = (HttpWebRequest)WebRequest.Create(_serviceUri);
            request.Method = "POST";
            request.ContentType = "application/json";
            using (var requestStream = await request.GetRequestStreamAsync())
            {
                requestStream.Write(dataToSend, 0, dataToSend.Length);
                requestStream.Flush();
            }
            var response = (await request.GetResponseAsync()) as HttpWebResponse;
            var responseText = await new StreamReader(response.GetResponseStream()).ReadToEndAsync();
            RecievedTextDataBlock.Text = responseText;
            AddCard(parseVCard(responseText));
        }

        private Contact parseVCard(string vcfCardText)
        {
            var parsedContact = new Contact();
            var regexName = new System.Text.RegularExpressions.Regex(@"(?<strElementFN>N):(?<strLastName>[^\n\r;]*);(?<strFirstName>[^\n\r]*)");
            var match = regexName.Match(vcfCardText);
            parsedContact.FirstName = match.Groups["strFirstName"].Value ?? "";
            parsedContact.LastName = match.Groups["strLastName"].Value ?? "";
            var regexPhone = new System.Text.RegularExpressions.Regex(@"(?<strElementTEL>TEL;WORK;VOICE):(?<strPhone>[^\n\r]*)");
            match = regexPhone.Match(vcfCardText);
            parsedContact.Phones.Add(new ContactPhone()
                                    {
                                        Number = match.Groups["strPhone"].Value
                                    });

            var regexEmail = new System.Text.RegularExpressions.Regex(@"(?<strElementEmail>EMAIL;PREF;INTERNET):(?<strEmail>[^\n\r]*)");
            match = regexEmail.Match(vcfCardText);
            parsedContact.Emails.Add(new ContactEmail()
                {
                    Address = match.Groups["strEmail"].Value,
                    Kind = ContactEmailKind.Work
                });

            parsedContact.Id = parsedContact.LastName + ";" + parsedContact.FirstName;

            return parsedContact;
        }

        private void AddCard(Contact contactCard)
        {
            var loader = ContactManager.ShowDelayLoadedContactCard(contactCard,
                                                        new Rect(500,500,100,100),
                                                        Placement.Below);
            loader.SetData(contactCard);
        }

        private async void BrowseButton_Click(object sender, RoutedEventArgs e)
        {
            var filePicker = new FileOpenPicker();
            filePicker.CommitButtonText = "Choose Card";
            filePicker.ViewMode = PickerViewMode.Thumbnail;
            filePicker.FileTypeFilter.Add(".jpg");
            filePicker.FileTypeFilter.Add(".png");
            filePicker.FileTypeFilter.Add(".bmp");
            _file = await filePicker.PickSingleFileAsync();

            await SetImageInHolder(_file);
        }
    }
}
