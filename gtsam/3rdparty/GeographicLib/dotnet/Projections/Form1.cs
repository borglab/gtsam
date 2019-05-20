/**
 * \file NETGeographicLib\Form1.cs
 * \brief Main Form for C# example
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using NETGeographicLib;

namespace Projections
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
            Text = "NETGeographicLib Demo - Version " + VersionInfo.GetString();

            // set up the tab control
            m_geodesicTabPage.Controls.Add(new GeodesicPanel());
            m_geocentricTabPage.Controls.Add(new GeocentricPanel());
            m_localCartesianPage.Controls.Add(new LocalCartesianPanel());
            m_albersPage.Controls.Add(new AlbersPanel());
            m_typeIIProjections.Controls.Add(new ProjectionsPanel());
            m_typeIIIProjPage.Controls.Add(new TypeIIIProjPanel());
            m_polarStereoPage.Controls.Add(new PolarStereoPanel());
            m_sphericalPage.Controls.Add(new SphericalHarmonicsPanel());
            m_ellipticPage.Controls.Add(new EllipticPanel());
            m_ellipsoidPage.Controls.Add(new EllipsoidPanel());
            m_miscPage.Controls.Add(new MiscPanel());
            m_geoidPage.Controls.Add(new GeoidPanel());
            m_gravityPage.Controls.Add(new GravityPanel());
            m_magneticPage.Controls.Add(new MagneticPanel());
            m_polyPage.Controls.Add(new PolyPanel());
            m_accumPage.Controls.Add(new AccumPanel());
            m_rhumbTabPage.Controls.Add(new RhumbPanel());
        }
    }
}
