using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Web;
using System.Web.Http;
using System.Web.Mvc;
using System.Web.Optimization;
using System.Web.Routing;

namespace BusinessCardReader.Service
{
    // Note: For instructions on enabling IIS6 or IIS7 classic mode, 
    // visit http://go.microsoft.com/?LinkId=9394801

    public class WebApiApplication : System.Web.HttpApplication
    {
        protected void Application_Start()
        {
            AreaRegistration.RegisterAllAreas();

            WebApiConfig.Register(GlobalConfiguration.Configuration);
            FilterConfig.RegisterGlobalFilters(GlobalFilters.Filters);
            RouteConfig.RegisterRoutes(RouteTable.Routes);
            BundleConfig.RegisterBundles(BundleTable.Bundles);
            AddBinToPath();
            AddTessDataEnvironmentVariable();
        }

        /// <summary>
        /// Add the bin folder to the path variable so that
        /// the native dlls are visible to the application.
        /// </summary>
        protected void AddBinToPath()
        {
            var binaryPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "bin", "NativeCode");
            var path = Environment.GetEnvironmentVariable("PATH") ?? "";

            if(!path.Split(Path.PathSeparator).Contains(binaryPath))
            {
                path = string.Join(Path.PathSeparator.ToString(), new string[] { path, binaryPath });
                Environment.SetEnvironmentVariable("PATH", path);
            }
        }

        protected void AddTessDataEnvironmentVariable()
        {
            var binaryPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "bin", "NativeCode","");
            Environment.SetEnvironmentVariable("TESSDATA_PREFIX", binaryPath);
        }

        protected void Application_End()
        {

        }
    }
}