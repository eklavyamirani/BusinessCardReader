using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Http;
using System.Text;
using System.Web.Http;

namespace BusinessCardReader.Service.Controllers
{
    public class ValuesController : ApiController
    {
        // GET api/values
        public IEnumerable<string> Get()
        {
            return new string[] { "value1", "value2" };
        }

        // GET api/values/5
        public string Get(int id)
        {
            return "value";
        }

        // POST api/values
        public HttpResponseMessage Post([FromBody]string value)
        {
            if(string.IsNullOrEmpty(value))
            {
                return Request.CreateResponse(HttpStatusCode.BadRequest);
            }
            //Assuming the value is UTF8 byte string
            System.Diagnostics.Trace.WriteLine(value.Length);
            var imageContent = (byte[])Newtonsoft.Json.JsonConvert.DeserializeObject("\"" + value + "\"",typeof(byte[]));
            //Call Some Async Method To initiate the processing
            try
            {
                var contactInformation = BusinessCardReader.Service.NativeWrapper.DetectText(imageContent);
                if(contactInformation == null)
                {
                    return Request.CreateResponse(HttpStatusCode.NoContent);
                }
                else
                {
                    var response = Request.CreateResponse(HttpStatusCode.OK);
                    var vcfCard = contactInformation.CreateVCFCard();
                    response.Content = new StringContent(vcfCard, Encoding.UTF8,"text/vcard");
                    return response;
                }
            }
            catch (System.BadImageFormatException e)
            {
                return Request.CreateResponse(HttpStatusCode.NotAcceptable);
            }
            catch (DllNotFoundException e)
            {
                return Request.CreateResponse(HttpStatusCode.NotImplemented);
            }
            return Request.CreateResponse(HttpStatusCode.OK);
        }

        // PUT api/values/5
        public void Put(int id, [FromBody]string value)
        {
        }

        // DELETE api/values/5
        public void Delete(int id)
        {
        }
    }
}