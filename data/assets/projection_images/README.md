# Affinity Designer Setup for Projection Images

This guide will walk you through the steps to ensure proper setup of Affinity Designer for creating images for the projection opperations.

## 1. Creating a new document

1. Open **Affinity Designer**.
2. Go to **File > New** or press `Ctrl + N`.
3. In the **New Document** dialog, under the **Color** section:
   - Check the box labeled **Transparent Background**.
4. Under the **Dimentions** section:
   - Set the **DPI** to 72.
4. Set the desired dimensions, resolution, and other document settings.
5. Click **Create** to open a new document with transparency enabled.

## 2. Working with transparency in existing documents

If you are working with an existing document and need to enable transparency:
1. Go to **Document Setup**.
2. Under the **Color** section:
   - Check the box labeled **Transparent Background**.
3. Under the **Dimentions** section:
   - Set the **DPI** to 72.

## 3. Ensuring Transparent Elements in the Design

- Make sure to leave areas where transparency is desired without any fills or layers.
- If you need transparent shapes or objects, use the **Fill Tool** (G) and set the fill to **None** or adjust the **Opacity** slider for partial transparency.

## 4. Exporting as a Single Transparent PNG (RGBA)

To export your design as a PNG with a transparent background:

1. Select the objects you wish to export (or the entire design).
2. Go to **File > Export**.
3. In the export dialog:
   - Select **PNG** as the format.
   - Under the **Area** dropdown, choose the page you are exporting. For instance  **Page 1**.
   - In the **Advanced** section, ensure that the **Matte** option is set to **None** (diagonal red line).
   - Optionally, embed an **ICC profile** if needed for color accuracy across devices.
4. Click **Export** and save your file to the desired location.

## 4. Exporting Multiple Artboards at Once 

1. **Switch to Export Persona**:
   - In Affinity Designer, click on the **Export Persona** icon at the top left (square with a diagonal arrow).

2. **Open the Slices Panel**:
   - If the **Slices** panel is not visible, go to **View > Studio > Slices** to enable it.

3. **Select Artboards**:
   - The **Slices** panel will automatically display your artboards as separate slices.

4. **Choose Export Format**:
   - Select the slices (artboards) you want to export.
   - In the export options, choose **PNG** as the format.

5. **Export Slices**:
   - Once all slices are selected, click **Export Slices** at the bottom of the panel. 
   - Choose the location to save the files, and each artboard will be saved as an individual PNG file.
   - Confirm the outputted image is *300 x 540* pixels.
