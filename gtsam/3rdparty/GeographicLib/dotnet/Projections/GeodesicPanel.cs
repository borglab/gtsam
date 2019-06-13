/**
 * \file NETGeographicLib\GeodesicPanel.cs
 * \brief NETGeographicLib.Geodesic example
 *
 * NETGeographicLib.Geodesic,
 * NETGeographicLib.GeodesicLine,
 * NETGeographicLib.GeodesicExact,
 * NETGeographicLib.GeodesicLineExact
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
using System.Drawing;
using System.Data;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using NETGeographicLib;

namespace Projections
{
    public partial class GeodesicPanel : UserControl
    {
        public string warning = "GeographicLib Error";
        public string warning2 = "Data Conversion Error";

        enum Function
        {
            Direct = 0,
            Inverse = 1
        };
        Function m_function = Function.Direct;

        enum Variable
        {
            Distance = 0,
            ArcLength = 2
        };
        Variable m_variable = Variable.Distance;

        enum Classes
        {
            GEODESIC = 0,
            GEODESICEXACT= 1,
            GEODESICLINE = 2,
            GEODESICLINEEXACT = 3
        };
        Classes m_class = Classes.GEODESIC;

        Geodesic m_geodesic = null;

        public GeodesicPanel()
        {
            InitializeComponent();
            m_tooltips.SetToolTip(button1, "Performs the selected function with the selected class");
            m_tooltips.SetToolTip(m_setButton, "Sets the ellipsoid attributes");
            m_tooltips.SetToolTip(m_validateButton, "Validates Geodesic, GeodesicExact, GeodesicLine, and GeodesicLineExact interfaces");
            try
            {
                m_geodesic = new Geodesic();
            }
            catch (GeographicErr err)
            {
                MessageBox.Show(err.Message, warning, MessageBoxButtons.OK, MessageBoxIcon.Error);
            }

            m_majorRadiusTextBox.Text = m_geodesic.MajorRadius.ToString();
            m_flatteningTextBox.Text = m_geodesic.Flattening.ToString();
            m_functionComboBox.SelectedIndex = 0;
            m_classComboBox.SelectedIndex = 0;
        }

        // gets the major radius and flattening and creates a new Geodesic
        private void OnSet(object sender, EventArgs e)
        {
            try
            {
                double radius = Double.Parse(m_majorRadiusTextBox.Text);
                double flattening = Double.Parse(m_flatteningTextBox.Text);
                m_geodesic = new Geodesic(radius, flattening);
            }
            catch (GeographicErr err)
            {
                MessageBox.Show(err.Message, warning, MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            catch (Exception err2)
            {
                MessageBox.Show(err2.Message, warning2, MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        // Gets the input parameters and calls the appropriate function
        private void OnForward(object sender, EventArgs e)
        {
            double origLatitude = 0.0, origLongitude = 0.0, origAzimuth = 0.0,
                distance = 0.0, finalLatitude = 0.0, finalLongitude = 0.0;
            // get & validate inputs
            try
            {
                if ( m_function == Function.Direct )
                {
                    distance = Double.Parse( m_variable == Variable.Distance ?
                        m_distanceTextBox.Text : m_ArcLengthTextBox.Text );
                    origAzimuth = Double.Parse( m_originAzimuthTextBox.Text );
                    if ( origAzimuth < -180.0 || origAzimuth > 180.0 )
                    {
                        m_originAzimuthTextBox.Focus();
                        throw new Exception( "Range Error: -180 <= initial azimuth <= 180 degrees" );
                    }
                }
                else
                {
                    finalLatitude = Double.Parse( m_finalLatitudeTextBox.Text );
                    if (finalLatitude < -90.0 || finalLatitude > 90.0)
                    {
                        m_finalLatitudeTextBox.Focus();
                        throw new Exception("Range Error: -90 <= final latitude <= 90 degrees");
                    }
                    finalLongitude = Double.Parse( m_finalLongitudeTextBox.Text );
                    if (finalLongitude < -540.0 || finalLongitude > 540.0)
                    {
                        m_finalLongitudeTextBox.Focus();
                        throw new Exception("Range Error: -540 <= final longitude <= 540 degrees");
                    }
                }
                origLatitude = Double.Parse( m_originLatitudeTextBox.Text );
                if (origLatitude < -90.0 || origLatitude > 90.0)
                {
                    m_originLatitudeTextBox.Focus();
                    throw new Exception("Range Error: -90 <= initial latitude <= 90 degrees");
                }
                origLongitude = Double.Parse(m_originLongitudeTextBox.Text);
                if (origLongitude < -540.0 || origLongitude > 540.0)
                {
                    m_originLongitudeTextBox.Focus();
                    throw new Exception("Range Error: -540 <= initial longitude <= 540 degrees");
                }
            }
            catch ( Exception xcpt )
            {
                MessageBox.Show(xcpt.Message, warning2, MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            // excute the appropriate function.
            double finalAzimuth = 0.0, reducedLength = 0.0, M12 = 0.0, M21 = 0.0,
                S12 = 0.0, arcDistance = 0.0;
            int sw = (int)m_function | (int)m_variable;
            if (sw == 3) sw = 1; // cases 1 & 3 are identical.
            try
            {
                switch (m_class)
                {
                    case Classes.GEODESIC:
                        switch (sw)
                        {
                            case 0: // function == Direct, variable == distance
                                m_ArcLengthTextBox.Text =
                                    m_geodesic.Direct(origLatitude, origLongitude, origAzimuth, distance,
                                        out finalLatitude, out finalLongitude, out finalAzimuth, out reducedLength,
                                        out M12, out M21, out S12).ToString();
                                m_finalLatitudeTextBox.Text = finalLatitude.ToString();
                                m_finalLongitudeTextBox.Text = finalLongitude.ToString();
                                break;
                            case 1: // function == Inverse, variable == distance
                                m_ArcLengthTextBox.Text =
                                    m_geodesic.Inverse(origLatitude, origLongitude, finalLatitude, finalLongitude,
                                        out distance, out origAzimuth, out finalAzimuth, out reducedLength, out M12,
                                        out M21, out S12).ToString();
                                m_distanceTextBox.Text = distance.ToString();
                                m_originAzimuthTextBox.Text = origAzimuth.ToString();
                                break;
                            case 2: // function == Direct, variable == arc length
                                m_geodesic.ArcDirect(origLatitude, origLongitude, origAzimuth, distance,
                                    out finalLatitude, out finalLongitude, out finalAzimuth, out arcDistance,
                                    out reducedLength, out M12, out M21, out S12);
                                m_distanceTextBox.Text = arcDistance.ToString();
                                m_finalLatitudeTextBox.Text = finalLatitude.ToString();
                                m_finalLongitudeTextBox.Text = finalLongitude.ToString();
                                break;
                        }
                        m_finalAzimuthTextBox.Text = finalAzimuth.ToString();
                        m_reducedLengthTextBox.Text = reducedLength.ToString();
                        m_M12TextBox.Text = M12.ToString();
                        m_M21TextBox.Text = M21.ToString();
                        m_S12TextBox.Text = S12.ToString();
                        break;
                    case Classes.GEODESICEXACT:
                        GeodesicExact ge = new GeodesicExact(m_geodesic.MajorRadius, m_geodesic.Flattening);
                        switch (sw)
                        {
                            case 0: // function == Direct, variable == distance
                                m_ArcLengthTextBox.Text =
                                    ge.Direct(origLatitude, origLongitude, origAzimuth, distance,
                                        out finalLatitude, out finalLongitude, out finalAzimuth, out reducedLength,
                                        out M12, out M21, out S12).ToString();
                                m_finalLatitudeTextBox.Text = finalLatitude.ToString();
                                m_finalLongitudeTextBox.Text = finalLongitude.ToString();
                                break;
                            case 1: // function == Inverse, variable == distance
                                m_ArcLengthTextBox.Text =
                                    ge.Inverse(origLatitude, origLongitude, finalLatitude, finalLongitude,
                                        out distance, out origAzimuth, out finalAzimuth, out reducedLength, out M12,
                                        out M21, out S12).ToString();
                                m_distanceTextBox.Text = distance.ToString();
                                m_originAzimuthTextBox.Text = origAzimuth.ToString();
                                break;
                            case 2: // function == Direct, variable == arc length
                                ge.ArcDirect(origLatitude, origLongitude, origAzimuth, distance,
                                    out finalLatitude, out finalLongitude, out finalAzimuth, out arcDistance,
                                    out reducedLength, out M12, out M21, out S12);
                                m_distanceTextBox.Text = arcDistance.ToString();
                                m_finalLatitudeTextBox.Text = finalLatitude.ToString();
                                m_finalLongitudeTextBox.Text = finalLongitude.ToString();
                                break;
                        }
                        m_finalAzimuthTextBox.Text = finalAzimuth.ToString();
                        m_reducedLengthTextBox.Text = reducedLength.ToString();
                        m_M12TextBox.Text = M12.ToString();
                        m_M21TextBox.Text = M21.ToString();
                        m_S12TextBox.Text = S12.ToString();
                        break;
                    case Classes.GEODESICLINE:
                        GeodesicLine gl = new GeodesicLine(m_geodesic, origLatitude, origLongitude, origAzimuth, Mask.ALL);
                        switch (sw)
                        {
                            case 0: // function == Direct, variable == distance
                                m_ArcLengthTextBox.Text =
                                    gl.Position(distance,
                                        out finalLatitude, out finalLongitude, out finalAzimuth, out reducedLength,
                                        out M12, out M21, out S12).ToString();
                                m_finalLatitudeTextBox.Text = finalLatitude.ToString();
                                m_finalLongitudeTextBox.Text = finalLongitude.ToString();
                                break;
                            case 1: // function == Inverse, variable == distance
                                throw new Exception("GeodesicLine does not have an Inverse function");
                            case 2: // function == Direct, variable == arc length
                                gl.ArcPosition(distance,
                                    out finalLatitude, out finalLongitude, out finalAzimuth, out arcDistance,
                                    out reducedLength, out M12, out M21, out S12);
                                m_distanceTextBox.Text = arcDistance.ToString();
                                m_finalLatitudeTextBox.Text = finalLatitude.ToString();
                                m_finalLongitudeTextBox.Text = finalLongitude.ToString();
                                break;
                        }
                        m_finalAzimuthTextBox.Text = finalAzimuth.ToString();
                        m_reducedLengthTextBox.Text = reducedLength.ToString();
                        m_M12TextBox.Text = M12.ToString();
                        m_M21TextBox.Text = M21.ToString();
                        m_S12TextBox.Text = S12.ToString();
                        break;
                    case Classes.GEODESICLINEEXACT:
                        GeodesicLineExact gle = new GeodesicLineExact(origLatitude, origLongitude, origAzimuth, Mask.ALL);
                        switch (sw)
                        {
                            case 0: // function == Direct, variable == distance
                                m_ArcLengthTextBox.Text =
                                    gle.Position(distance,
                                        out finalLatitude, out finalLongitude, out finalAzimuth, out reducedLength,
                                        out M12, out M21, out S12).ToString();
                                m_finalLatitudeTextBox.Text = finalLatitude.ToString();
                                m_finalLongitudeTextBox.Text = finalLongitude.ToString();
                                break;
                            case 1: // function == Inverse, variable == distance
                                throw new Exception("GeodesicLineExact does not have an Inverse function");
                            case 2: // function == Direct, variable == arc length
                                gle.ArcPosition(distance,
                                    out finalLatitude, out finalLongitude, out finalAzimuth, out arcDistance,
                                    out reducedLength, out M12, out M21, out S12);
                                m_distanceTextBox.Text = arcDistance.ToString();
                                m_finalLatitudeTextBox.Text = finalLatitude.ToString();
                                m_finalLongitudeTextBox.Text = finalLongitude.ToString();
                                break;
                        }
                        m_finalAzimuthTextBox.Text = finalAzimuth.ToString();
                        m_reducedLengthTextBox.Text = reducedLength.ToString();
                        m_M12TextBox.Text = M12.ToString();
                        m_M21TextBox.Text = M21.ToString();
                        m_S12TextBox.Text = S12.ToString();
                        break;
                }
            }
            catch (Exception err)
            {
                MessageBox.Show(err.Message, warning, MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
        // gui stuff
        private void OnDistance(object sender, EventArgs e)
        {
            m_distanceTextBox.ReadOnly = false;
            m_ArcLengthTextBox.ReadOnly = true;
            m_variable = Variable.Distance;
        }

        // gui stuff
        private void OnArcLength(object sender, EventArgs e)
        {
            m_distanceTextBox.ReadOnly = true;
            m_ArcLengthTextBox.ReadOnly = false;
            m_variable = Variable.ArcLength;
        }

        // gui stuff
        private void OnFunction(object sender, EventArgs e)
        {
            m_function = (Function)m_functionComboBox.SelectedIndex;
            switch (m_function)
            {
                case Function.Direct:
                    m_distanceTextBox.ReadOnly = m_variable == Variable.ArcLength;
                    m_ArcLengthTextBox.ReadOnly = m_variable == Variable.Distance;
                    m_originAzimuthTextBox.ReadOnly = false;
                    m_finalLatitudeTextBox.ReadOnly = true;
                    m_finalLongitudeTextBox.ReadOnly = true;
                    break;
                case Function.Inverse:
                    m_distanceTextBox.ReadOnly = true;
                    m_ArcLengthTextBox.ReadOnly = true;
                    m_originAzimuthTextBox.ReadOnly = true;
                    m_finalLatitudeTextBox.ReadOnly = false;
                    m_finalLongitudeTextBox.ReadOnly = false;
                    break;
            }
        }
        // gui stuff
        private void OnClassChanged(object sender, EventArgs e)
        {
            m_class = (Classes)m_classComboBox.SelectedIndex;
        }

        // a simple validation function...does not change GUI elements
        private void OnValidate(object sender, EventArgs e)
        {
            double finalAzimuth = 0.0, reducedLength = 0.0, M12 = 0.0, M21 = 0.0,
                S12 = 0.0, arcDistance = 0.0, finalLatitude = 0.0, finalLongitude = 0.0,
                distance = 0.0;
            try
            {
                Geodesic g = new Geodesic();
                g = new Geodesic(g.MajorRadius, g.Flattening);
                arcDistance = g.Direct(32.0, -86.0, 45.0, 20000.0, out finalLatitude, out finalLongitude,
                    out finalAzimuth, out reducedLength, out M12, out M21,
                    out S12);
                double flat = 0.0, flon = 0.0, faz = 0.0, frd = 0.0, fm12 = 0.0, fm21 = 0.0, fs12 = 0.0, fad = 0.0;
                fad = g.Direct(32.0, -86.0, 45.0, 20000.0, out flat, out flon);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude)
                    throw new Exception("Geodesic.Direct #1 failed");
                fad = g.Direct(32.0, -86.0, 45.0, 20000.0, out flat, out flon, out faz);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth)
                    throw new Exception("Geodesic.Direct #2 failed");
                fad = g.Direct(32.0, -86.0, 45.0, 20000.0, out flat, out flon, out faz, out frd);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || frd != reducedLength)
                    throw new Exception("Geodesic.Direct #3 failed");
                fad = g.Direct(32.0, -86.0, 45.0, 20000.0, out flat, out flon, out faz, out fm12, out fm21);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || fm12 != M12 || fm21 != M21)
                    throw new Exception("Geodesic.Direct #4 failed");
                fad = g.Direct(32.0, -86.0, 45.0, 20000.0, out flat, out flon, out faz, out frd, out fm12, out fm21);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || frd != reducedLength || fm12 != M12 || fm21 != M21)
                    throw new Exception("Geodesic.Direct #5 failed");
                double outd = 0.0;
                fad = g.GenDirect(32.0, -86.0, 45.0, false, 20000.0, Geodesic.mask.ALL, out flat, out flon, out faz, out outd, out frd, out fm12, out fm21, out fs12);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || frd != reducedLength || fm12 != M12 || fm21 != M21 ||
                    outd != 20000.0 || fs12 != S12)
                    throw new Exception("Geodesic.GenDirect (false) failed");
                g.ArcDirect(32.0, -86.0, 45.0, 1.0, out finalLatitude, out finalLongitude, out finalAzimuth,
                    out arcDistance, out reducedLength, out M12, out M21, out S12);
                g.ArcDirect(32.0, -86.0, 45.0, 1.0, out flat, out flon);
                if (flat != finalLatitude || flon != finalLongitude)
                    throw new Exception("Geodesic.ArcDirect #1 failed");
                g.ArcDirect(32.0, -86.0, 45.0, 1.0, out flat, out flon, out faz);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth)
                    throw new Exception("Geodesic.ArcDirect #2 failed");
                g.ArcDirect(32.0, -86.0, 45.0, 1.0, out flat, out flon, out faz, out fad);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    fad != arcDistance)
                    throw new Exception("Geodesic.ArcDirect #3 failed");
                g.ArcDirect(32.0, -86.0, 45.0, 1.0, out flat, out flon, out faz, out fad, out frd);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    fad != arcDistance || frd != reducedLength)
                    throw new Exception("Geodesic.ArcDirect #4 failed");
                g.ArcDirect(32.0, -86.0, 45.0, 1.0, out flat, out flon, out faz, out fad, out fm12, out fm21);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    fad != arcDistance || fm12 != M12 || fm21 != M21)
                    throw new Exception("Geodesic.ArcDirect #5 failed");
                fad = g.GenDirect(32.0, -86.0, 45.0, true, 1.0, Geodesic.mask.ALL, out flat, out flon,
                    out faz, out outd, out frd, out fm12, out fm21, out fs12);
                if (outd != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || frd != reducedLength || fm12 != M12 || fm21 != M21 ||
                    fs12 != S12 || fad != 1.0)
                    throw new Exception("Geodesic.GenDirect (true) failed");
                double initAzimuth = 0.0, iaz = 0.0;
                arcDistance = g.Inverse(32.0, -86.0, 33.0, -87.0, out distance, out initAzimuth, out finalAzimuth,
                    out reducedLength, out M12, out M21, out S12);
                fad = g.Inverse(32.0, -86.0, 33.0, -87.0, out outd);
                if (fad != arcDistance || outd != distance)
                    throw new Exception("Geodesic.Inverse #1 failed");
                fad = g.Inverse(32.0, -86.0, 33.0, -87.0, out iaz, out faz);
                if (fad != arcDistance || iaz != initAzimuth || faz != finalAzimuth)
                    throw new Exception("Geodesic.Inverse #2 failed");
                fad = g.Inverse(32.0, -86.0, 33.0, -87.0, out outd, out iaz, out faz);
                if (fad != arcDistance || outd != distance || faz != finalAzimuth ||
                    outd != distance)
                    throw new Exception("Geodesic.Inverse #3 failed");
                fad = g.Inverse(32.0, -86.0, 33.0, -87.0, out outd, out iaz, out faz, out frd);
                if (fad != arcDistance || outd != distance || faz != finalAzimuth ||
                    outd != distance || frd != reducedLength)
                    throw new Exception("Geodesic.Inverse #4 failed");
                fad = g.Inverse(32.0, -86.0, 33.0, -87.0, out outd, out iaz, out faz, out fm12, out fm21 );
                if (fad != arcDistance || outd != distance || faz != finalAzimuth ||
                    outd != distance || fm12 != M12 || fm21 != M21 )
                    throw new Exception("Geodesic.Inverse #5 failed");
                fad = g.Inverse(32.0, -86.0, 33.0, -87.0, out outd, out iaz, out faz, out frd, out fm12, out fm21);
                if (fad != arcDistance || outd != distance || faz != finalAzimuth ||
                    outd != distance || fm12 != M12 || fm21 != M21 || frd != reducedLength)
                    throw new Exception("Geodesic.Inverse #6 failed");
                GeodesicLine gl = g.Line(32.0, -86.0, 45.0, Mask.ALL);
                gl = g.InverseLine(32.0, -86.0, 33.0, -87.0, Mask.ALL);
                gl = g.DirectLine(32.0, -86.0, 45.0, 10000.0, Mask.ALL);
                gl = g.ArcDirectLine(32.0, -86.0, 45.0, 10000.0, Mask.ALL);
                gl = new GeodesicLine(32.0, -86.0, 45.0, Mask.ALL);
                gl = new GeodesicLine(g, 32.0, -86.0, 45.0, Mask.ALL);
                arcDistance = gl.Position(10000.0, out finalLatitude, out finalLongitude, out finalAzimuth,
                    out reducedLength, out M12, out M21, out S12);
                fad = gl.Position(10000.0, out flat, out flon);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude)
                    throw new Exception("GeodesicLine.Position #1 failed");
                fad = gl.Position(10000.0, out flat, out flon, out faz);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth)
                    throw new Exception("GeodesicLine.Position #2 failed");
                fad = gl.Position(10000.0, out flat, out flon, out faz, out frd);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || frd != reducedLength)
                    throw new Exception("GeodesicLine.Position #3 failed");
                fad = gl.Position(10000.0, out flat, out flon, out faz, out fm12, out fm21);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || fm12 != M12 || fm21 != M21)
                    throw new Exception("GeodesicLine.Position #4 failed");
                fad = gl.Position(10000.0, out flat, out flon, out faz, out frd, out fm12, out fm21);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || fm12 != M12 || fm21 != M21 || frd != reducedLength )
                    throw new Exception("GeodesicLine.Position #5 failed");
                fad = gl.GenPosition(false, 10000.0, GeodesicLine.mask.ALL, out flat, out flon, out faz, out outd, out frd, out fm12, out fm21, out fs12);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || outd != 10000.0 || fm12 != M12 || fm21 != M21 ||
                    frd != reducedLength || fs12 != S12 )
                    throw new Exception("GeodesicLine.GenPosition (false) failed");
                gl.ArcPosition(1.0, out finalLatitude, out finalLongitude, out finalAzimuth,
                    out distance, out reducedLength, out M12, out M21, out S12);
                gl.ArcPosition(1.0, out flat, out flon);
                if (flat != finalLatitude || flon != finalLongitude)
                    throw new Exception("GeodesicLine.ArcPosition #1 failed");
                gl.ArcPosition(1.0, out flat, out flon, out faz);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth)
                    throw new Exception("GeodesicLine.ArcPosition #2 failed");
                gl.ArcPosition(1.0, out flat, out flon, out faz, out outd);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    outd != distance)
                    throw new Exception("GeodesicLine.ArcPosition #3 failed");
                gl.ArcPosition(1.0, out flat, out flon, out faz, out outd, out frd);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    outd != distance || frd != reducedLength)
                    throw new Exception("GeodesicLine.ArcPosition #4 failed");
                gl.ArcPosition(1.0, out flat, out flon, out faz, out outd, out fm12, out fm21);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    outd != distance || fm12 != M12 || fm21 != M21)
                    throw new Exception("GeodesicLine.ArcPosition #5 failed");
                gl.ArcPosition(1.0, out flat, out flon, out faz, out outd, out frd, out fm12, out fm21);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    outd != distance || fm12 != M12 || fm21 != M21 || frd != reducedLength)
                    throw new Exception("GeodesicLine.ArcPosition #6 failed");
                fad = gl.GenPosition(true, 1.0, GeodesicLine.mask.ALL, out flat, out flon, out faz, out outd, out frd, out fm12, out fm21, out fs12);
                if (fad != 1.0 || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || outd != distance || fm12 != M12 || fm21 != M21 ||
                    frd != reducedLength || fs12 != S12)
                    throw new Exception("GeodesicLine.GenPosition (false) failed");

                GeodesicExact ge = new GeodesicExact();
                ge = new GeodesicExact(g.MajorRadius, g.Flattening);
                arcDistance = ge.Direct(32.0, -86.0, 45.0, 20000.0, out finalLatitude, out finalLongitude,
                    out finalAzimuth, out reducedLength, out M12, out M21,
                    out S12);
                fad = ge.Direct(32.0, -86.0, 45.0, 20000.0, out flat, out flon);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude)
                    throw new Exception("GeodesicExact.Direct #1 failed");
                fad = ge.Direct(32.0, -86.0, 45.0, 20000.0, out flat, out flon, out faz);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth)
                    throw new Exception("GeodesicExact.Direct #2 failed");
                fad = ge.Direct(32.0, -86.0, 45.0, 20000.0, out flat, out flon, out faz, out frd);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || frd != reducedLength)
                    throw new Exception("GeodesicExact.Direct #3 failed");
                fad = ge.Direct(32.0, -86.0, 45.0, 20000.0, out flat, out flon, out faz, out fm12, out fm21);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || fm12 != M12 || fm21 != M21)
                    throw new Exception("GeodesicExact.Direct #4 failed");
                fad = ge.Direct(32.0, -86.0, 45.0, 20000.0, out flat, out flon, out faz, out frd, out fm12, out fm21);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || frd != reducedLength || fm12 != M12 || fm21 != M21)
                    throw new Exception("GeodesicExact.Direct #5 failed");
                fad = ge.GenDirect(32.0, -86.0, 45.0, false, 20000.0, GeodesicExact.mask.ALL, out flat, out flon, out faz, out outd, out frd, out fm12, out fm21, out fs12);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || frd != reducedLength || fm12 != M12 || fm21 != M21 ||
                    outd != 20000.0 || fs12 != S12)
                    throw new Exception("GeodesicExact.GenDirect (false) failed");
                ge.ArcDirect(32.0, -86.0, 45.0, 1.0, out finalLatitude, out finalLongitude, out finalAzimuth,
                    out arcDistance, out reducedLength, out M12, out M21, out S12);
                ge.ArcDirect(32.0, -86.0, 45.0, 1.0, out flat, out flon);
                if (flat != finalLatitude || flon != finalLongitude)
                    throw new Exception("GeodesicExact.ArcDirect #1 failed");
                ge.ArcDirect(32.0, -86.0, 45.0, 1.0, out flat, out flon, out faz);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth)
                    throw new Exception("GeodesicExact.ArcDirect #2 failed");
                ge.ArcDirect(32.0, -86.0, 45.0, 1.0, out flat, out flon, out faz, out fad);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    fad != arcDistance)
                    throw new Exception("GeodesicExact.ArcDirect #3 failed");
                ge.ArcDirect(32.0, -86.0, 45.0, 1.0, out flat, out flon, out faz, out fad, out frd);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    fad != arcDistance || frd != reducedLength)
                    throw new Exception("GeodesicExact.ArcDirect #4 failed");
                ge.ArcDirect(32.0, -86.0, 45.0, 1.0, out flat, out flon, out faz, out fad, out fm12, out fm21);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    fad != arcDistance || fm12 != M12 || fm21 != M21)
                    throw new Exception("GeodesicExact.ArcDirect #5 failed");
                fad = ge.GenDirect(32.0, -86.0, 45.0, true, 1.0, GeodesicExact.mask.ALL, out flat, out flon, out faz, out outd, out frd, out fm12, out fm21, out fs12);
                if (outd != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || frd != reducedLength || fm12 != M12 || fm21 != M21 ||
                    fad != 1.0 || fs12 != S12)
                    throw new Exception("GeodesicExact.GenDirect (true) failed");
                arcDistance = ge.Inverse(32.0, -86.0, 33.0, -87.0, out distance, out initAzimuth, out finalAzimuth,
                    out reducedLength, out M12, out M21, out S12);
                fad = ge.Inverse(32.0, -86.0, 33.0, -87.0, out outd);
                if (fad != arcDistance || outd != distance)
                    throw new Exception("GeodesicExact.Inverse #1 failed");
                fad = ge.Inverse(32.0, -86.0, 33.0, -87.0, out iaz, out faz);
                if (fad != arcDistance || iaz != initAzimuth || faz != finalAzimuth)
                    throw new Exception("GeodesicExact.Inverse #2 failed");
                fad = ge.Inverse(32.0, -86.0, 33.0, -87.0, out outd, out iaz, out faz);
                if (fad != arcDistance || outd != distance || faz != finalAzimuth ||
                    outd != distance)
                    throw new Exception("GeodesicExact.Inverse #3 failed");
                fad = ge.Inverse(32.0, -86.0, 33.0, -87.0, out outd, out iaz, out faz, out frd);
                if (fad != arcDistance || outd != distance || faz != finalAzimuth ||
                    outd != distance || frd != reducedLength)
                    throw new Exception("GeodesicExact.Inverse #4 failed");
                fad = ge.Inverse(32.0, -86.0, 33.0, -87.0, out outd, out iaz, out faz, out fm12, out fm21);
                if (fad != arcDistance || outd != distance || faz != finalAzimuth ||
                    outd != distance || fm12 != M12 || fm21 != M21)
                    throw new Exception("GeodesicExact.Inverse #5 failed");
                fad = ge.Inverse(32.0, -86.0, 33.0, -87.0, out outd, out iaz, out faz, out frd, out fm12, out fm21);
                if (fad != arcDistance || outd != distance || faz != finalAzimuth ||
                    outd != distance || fm12 != M12 || fm21 != M21 || frd != reducedLength)
                    throw new Exception("GeodesicExact.Inverse #6 failed");
                GeodesicLineExact gle = ge.Line(32.0, -86.0, 45.0, Mask.ALL);
                gle = ge.InverseLine(32.0, -86.0, 33.0, -87.0, Mask.ALL);
                gle = ge.DirectLine(32.0, -86.0, 45.0, 10000.0, Mask.ALL);
                gle = ge.ArcDirectLine(32.0, -86.0, 45.0, 10000.0, Mask.ALL);
                gle = new GeodesicLineExact(32.0, -86.0, 45.0, Mask.ALL);
                gle = new GeodesicLineExact(ge, 32.0, -86.0, 45.0, Mask.ALL);
                arcDistance = gle.Position(10000.0, out finalLatitude, out finalLongitude, out finalAzimuth,
                    out reducedLength, out M12, out M21, out S12);
                fad = gle.Position(10000.0, out flat, out flon);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude)
                    throw new Exception("GeodesicLineExact.Position #1 failed");
                fad = gle.Position(10000.0, out flat, out flon, out faz);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth)
                    throw new Exception("GeodesicLineExact.Position #2 failed");
                fad = gle.Position(10000.0, out flat, out flon, out faz, out frd);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || frd != reducedLength)
                    throw new Exception("GeodesicLineExact.Position #3 failed");
                fad = gle.Position(10000.0, out flat, out flon, out faz, out fm12, out fm21);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || fm12 != M12 || fm21 != M21)
                    throw new Exception("GeodesicLineExact.Position #4 failed");
                fad = gle.Position(10000.0, out flat, out flon, out faz, out frd, out fm12, out fm21);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || fm12 != M12 || fm21 != M21 || frd != reducedLength)
                    throw new Exception("GeodesicLineExact.Position #5 failed");
                fad = gle.GenPosition(false, 10000.0, GeodesicLineExact.mask.ALL, out flat, out flon, out faz, out outd, out frd, out fm12, out fm21, out fs12);
                if (fad != arcDistance || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || outd != 10000.0 || fm12 != M12 || fm21 != M21 ||
                    frd != reducedLength || fs12 != S12)
                    throw new Exception("GeodesicLineExact.GenPosition (false) failed");
                gle.ArcPosition(1.0, out finalLatitude, out finalLongitude, out finalAzimuth,
                    out distance, out reducedLength, out M12, out M21, out S12);
                gle.ArcPosition(1.0, out flat, out flon);
                if (flat != finalLatitude || flon != finalLongitude)
                    throw new Exception("GeodesicLineExact.ArcPosition #1 failed");
                gle.ArcPosition(1.0, out flat, out flon, out faz);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth)
                    throw new Exception("GeodesicLineExact.ArcPosition #2 failed");
                gle.ArcPosition(1.0, out flat, out flon, out faz, out outd);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    outd != distance)
                    throw new Exception("GeodesicLineExact.ArcPosition #3 failed");
                gle.ArcPosition(1.0, out flat, out flon, out faz, out outd, out frd);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    outd != distance || frd != reducedLength)
                    throw new Exception("GeodesicLineExact.ArcPosition #4 failed");
                gle.ArcPosition(1.0, out flat, out flon, out faz, out outd, out fm12, out fm21);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    outd != distance || fm12 != M12 || fm21 != M21)
                    throw new Exception("GeodesicLineExact.ArcPosition #5 failed");
                gle.ArcPosition(1.0, out flat, out flon, out faz, out outd, out frd, out fm12, out fm21);
                if (flat != finalLatitude || flon != finalLongitude || faz != finalAzimuth ||
                    outd != distance || fm12 != M12 || fm21 != M21 || frd != reducedLength)
                    throw new Exception("GeodesicLineExact.ArcPosition #6 failed");
                fad = gle.GenPosition(true, 1.0, GeodesicLineExact.mask.ALL, out flat, out flon, out faz, out outd, out frd, out fm12, out fm21, out fs12);
                if (fad != 1.0 || flat != finalLatitude || flon != finalLongitude ||
                    faz != finalAzimuth || outd != distance || fm12 != M12 || fm21 != M21 ||
                    frd != reducedLength || fs12 != S12)
                    throw new Exception("GeodesicLineExact.GenPosition (false) failed");
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Interface Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            MessageBox.Show("No errors detected", "Interfaces OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
        }
    }
}
