/**
 * \file NETGeographicLib\GeoidPanel.cs
 * \brief NETGeographicLib.Geoid example
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

using System.IO;

namespace Projections
{
    public partial class GeoidPanel : UserControl
    {
        Geoid m_geoid = null;
        string m_path;
        string m_fileName;

        public GeoidPanel()
        {
            InitializeComponent();
            m_threadSafeCheckBox.Checked = true;
        }

        private void OnSelectFile(object sender, EventArgs e)
        {
            OpenFileDialog dlg = new OpenFileDialog();
            dlg.Filter = "Geoid File (*.pgm)|*.pgm";
            dlg.DefaultExt = "pgm";
            dlg.Title = "Open Geoid File";

            if (dlg.ShowDialog() == DialogResult.Cancel) return;

            m_path = dlg.FileName.Substring(0, dlg.FileName.LastIndexOf('\\')).Replace('\\', '/');
            int length = dlg.FileName.LastIndexOf('.') - dlg.FileName.LastIndexOf('\\') - 1;
            m_fileName = dlg.FileName.Substring(dlg.FileName.LastIndexOf('\\') + 1, length);

            try
            {
                m_geoid = new Geoid(m_fileName, m_path, true, m_threadSafeCheckBox.Checked);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            m_convertEllipsodButton.Enabled = m_convertGeoidButton.Enabled = m_heightButton.Enabled = m_validateButton.Enabled = true;
            m_geoidFileNameTextBox.Text = dlg.FileName;
            m_dateTimeTextBox.Text = m_geoid.DateTime;
            m_descriptionTextBox.Text = m_geoid.Description;
            m_majorRadiusTextBox.Text = m_geoid.MajorRadius.ToString();
            m_flatteningTtextBox.Text = m_geoid.Flattening.ToString();
        }

        private void OnThreadSafe(object sender, EventArgs e)
        {
            if (m_geoidFileNameTextBox.Text.Length > 0)
            {
                try
                {
                    m_geoid = new Geoid(m_fileName, m_path, true, m_threadSafeCheckBox.Checked);
                }
                catch (Exception xcpt)
                {
                    MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }
            }
            m_cacheButton.Enabled = !m_threadSafeCheckBox.Checked;
            m_northTextBox.ReadOnly = m_southTextBox.ReadOnly = m_eastTextBox.ReadOnly = m_westTextBox.ReadOnly = m_threadSafeCheckBox.Checked;
        }

        private void OnCache(object sender, EventArgs e)
        {
            if (m_geoid == null) return;

            try
            {
                double south = Double.Parse(m_southTextBox.Text);
                double north = Double.Parse(m_northTextBox.Text);
                double west = Double.Parse(m_westTextBox.Text);
                double east = Double.Parse(m_eastTextBox.Text);
                m_geoid.CacheArea(south, west, north, east);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnConvertEllipsod(object sender, EventArgs e)
        {
            try
            {
                double lat = Double.Parse(m_latitudeTextBox.Text);
                double lon = Double.Parse(m_longitudeTextBox.Text);
                double h = Double.Parse(m_ellipsoidTextBox.Text);
                m_geoidTextBox.Text = m_geoid.ConvertHeight(lat, lon, h, Geoid.ConvertFlag.ELLIPSOIDTOGEOID).ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnConvertGeoid(object sender, EventArgs e)
        {
            try
            {
                double lat = Double.Parse(m_latitudeTextBox.Text);
                double lon = Double.Parse(m_longitudeTextBox.Text);
                double h = Double.Parse(m_geoidTextBox.Text);
                m_ellipsoidTextBox.Text = m_geoid.ConvertHeight(lat, lon, h, Geoid.ConvertFlag.GEOIDTOELLIPSOID).ToString();
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnHeight(object sender, EventArgs e)
        {
            try
            {
                double lat = Double.Parse(m_latitudeTextBox.Text);
                double lon = Double.Parse(m_longitudeTextBox.Text);
                m_ellipsoidTextBox.Text = m_geoid.Height(lat, lon).ToString();
                m_geoidTextBox.Text = "0";
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void OnValidate(object sender, EventArgs e)
        {
            try
            {
                Geoid g = new Geoid(m_fileName, m_path, false, false);
                g.CacheArea(20.0, -30.0, 30.0, -20.0);
                g.CacheAll();
                double h2 = g.Height(32.0, -60.0);
                g.ConvertHeight(32.0, -60.0, 100.0, Geoid.ConvertFlag.ELLIPSOIDTOGEOID);
                MessageBox.Show("No errors detected", "OK", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
            catch (Exception xcpt)
            {
                MessageBox.Show(xcpt.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
    }
}
