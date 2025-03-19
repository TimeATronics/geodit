"""
    This file is part of the Geodit distribution (https://github.com/TimeATronics/geodit).
    Copyright (c) 2024-2025 Aradhya Chakrabarti

    This program is free software: you can redistribute it and/or modify  
    it under the terms of the GNU General Public License as published by  
    the Free Software Foundation, version 3.

    This program is distributed in the hope that it will be useful, but 
    WITHOUT ANY WARRANTY; without even the implied warranty of 
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
    General Public License for more details.

    You should have received a copy of the GNU General Public License 
    along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
import sys
import os
import json
import threading
import http.server
import socketserver
import geopandas as gpd
import pandas as pd
import rasterio
from rasterio.warp import transform_bounds
from urllib.parse import unquote
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QFileDialog,
    QAction, QSplashScreen, QInputDialog, QMessageBox
)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPixmap, QIcon
from shapely.geometry import shape

script_dir = os.path.dirname(os.path.abspath(__file__))
splash_screen_path = os.path.join(script_dir, 'img', 'splash_screen.png')
icon_path = os.path.join(script_dir, 'img', 'icon.png')

class CORSRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, root_dir, **kwargs):
        self.root_dir = root_dir
        super().__init__(*args, **kwargs)

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        super().end_headers()

    def translate_path(self, path):
        path = unquote(path).lstrip("/")
        full_path = os.path.join(self.root_dir, path)
        full_path = os.path.normpath(full_path)
        print(f"Resolving path: {full_path}")
        if os.path.exists(full_path):
            return full_path
        else:
            return super().translate_path(path)

class GeoTIFFServer(threading.Thread):
    def __init__(self, root_dir, port=8000):
        super().__init__(daemon=True)
        self.port = port
        self.root_dir = root_dir

    def run(self):
        handler = lambda *args, **kwargs: CORSRequestHandler(*args, root_dir=self.root_dir, **kwargs)
        with socketserver.TCPServer(("localhost", self.port), handler) as httpd:
            print(f"Serving files from {self.root_dir} at http://localhost:{self.port}/ with unrestricted access")
            httpd.serve_forever()

class MapViewer(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("PyGeodit v0.1")
        self.setWindowIcon(QIcon(icon_path))
        self.setGeometry(100, 100, 900, 600)

        self.current_crs = "EPSG:4326"  # Default CRS
        self.current_shapefile = None
        self.menu_bar = self.menuBar()
        self.root_directory = None
        self.root_directory = self.get_root_directory()
        file_menu = self.menu_bar.addMenu("File")
        load_shp_action = QAction("Open Shapefile", self)
        load_shp_action.setShortcut("Ctrl+O")
        load_shp_action.triggered.connect(self.load_shapefile)
        file_menu.addAction(load_shp_action)

        save_shp_action = QAction("Save Shapefile", self)
        save_shp_action.setShortcut("Ctrl+S")
        save_shp_action.triggered.connect(self.save_shapefile)
        file_menu.addAction(save_shp_action)

        load_tiff_action = QAction("Open GeoTIFF", self)
        load_tiff_action.setShortcut("Ctrl+T")
        load_tiff_action.triggered.connect(self.load_geotiff)
        file_menu.addAction(load_tiff_action)

        set_crs_action = QAction("Set CRS", self)
        set_crs_action.triggered.connect(self.set_crs)
        file_menu.addAction(set_crs_action)

        metadata_action = QAction("Show Shapefile Metadata", self)
        metadata_action.triggered.connect(self.show_metadata)
        file_menu.addAction(metadata_action)

        exit_action = QAction("Exit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        edit_menu = self.menu_bar.addMenu("Edit")
        undo_action = QAction("Undo", self)
        undo_action.setShortcut("Ctrl+Z")
        undo_action.triggered.connect(self.undo_map)
        edit_menu.addAction(undo_action)

        redo_action = QAction("Redo", self)
        redo_action.setShortcut("Ctrl+Y")
        redo_action.triggered.connect(self.redo_map)
        edit_menu.addAction(redo_action)

        clear_map_action = QAction("Clear Map", self)
        clear_map_action.setShortcut("Ctrl+Shift+C")
        clear_map_action.triggered.connect(self.clear_map)
        edit_menu.addAction(clear_map_action)

        view_menu = self.menu_bar.addMenu("View")
        zoom_in_action = QAction("Zoom In", self)
        zoom_in_action.setShortcut("Ctrl++")
        zoom_in_action.triggered.connect(self.zoom_in)
        zoom_out_action = QAction("Zoom Out", self)
        zoom_out_action.setShortcut("Ctrl+-")
        zoom_out_action.triggered.connect(self.zoom_out)
        view_menu.addAction(zoom_in_action)
        view_menu.addAction(zoom_out_action)
        self.dark_mode_action = QAction("Enable Dark Mode", self)
        self.dark_mode_action.triggered.connect(self.toggle_dark_mode)
        view_menu.addAction(self.dark_mode_action)

        help_menu = self.menu_bar.addMenu("Help")
        about_action = QAction("About", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

        help_action = QAction("Help", self)
        help_action.setShortcut("F1")
        help_action.triggered.connect(self.show_help)
        help_menu.addAction(help_action)

        # Layout
        layout = QVBoxLayout()
        self.webView = QWebEngineView()
        layout.addWidget(self.webView)

        leaflet_map = self.generate_map()
        self.webView.setHtml(leaflet_map)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)
        self.tiff_path = None
        if self.root_directory:
            # Create and start the file server with the specified root directory
            server = GeoTIFFServer(root_dir=self.root_directory, port=8000)
            server.start()
            print(f"Server started at http://localhost:8000/ with root directory: {self.root_directory}")
        else:
            print("No root directory selected. Server not started.")

    def get_root_directory(self):
        root_dir = QFileDialog.getExistingDirectory(None, "Select Root Directory", os.getcwd())
        if not root_dir:
            return None
        self.root_directory = root_dir
        return root_dir

    def generate_map(self):
        return """
        <!DOCTYPE html>
        <html>
        <head>
            <title>OpenStreetMap with Drawing Tools</title>
            <meta charset="utf-8" />
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
            <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.css" />
            <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.js"></script>
            <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/leaflet-measure/dist/leaflet-measure.css" />
            <script src="https://cdn.jsdelivr.net/npm/leaflet-measure/dist/leaflet-measure.js"></script>
            <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet.geometryutil/0.9.3/leaflet.geometryutil.js"></script>
            </head>
        <body>
            <div id="map" style="width: 100%; height: 100vh;"></div>
            <script>
                var map = L.map('map').setView([20, 0], 2);
                L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                    attribution: '&copy; OpenStreetMap contributors'
                }).addTo(map);

                // Create a div for displaying coordinates & pixel values
                var coordDiv = document.createElement('div');
                coordDiv.style.position = 'absolute';
                coordDiv.style.bottom = '10px';
                coordDiv.style.left = '10px';
                coordDiv.style.background = 'rgba(255, 255, 255, 0.7)';
                coordDiv.style.padding = '5px';
                coordDiv.style.borderRadius = '5px';
                coordDiv.style.fontSize = '14px';
                coordDiv.style.zIndex = '1000';
                coordDiv.innerHTML = 'Hover over map for coordinates';
                document.body.appendChild(coordDiv);

                // Function to update the coordinate display
                function updateCoordinates(lat, lon, pixelValue = null) {
                    if (pixelValue !== null) {
                        coordDiv.innerHTML = `Lat: ${lat.toFixed(6)}, Lng: ${lon.toFixed(6)}<br>Pixel Value: ${pixelValue}`;
                    } else {
                        coordDiv.innerHTML = `Lat: ${lat.toFixed(6)}, Lng: ${lon.toFixed(6)}`;
                    }
                }

                // Update coordinates and pixel values on mouse move
                map.on('mousemove', async function (e) {
                    let lat = e.latlng.lat;
                    let lon = e.latlng.lng;

                    // If GeoTIFF is loaded, try to get the pixel value
                    if (window.georaster && typeof geoblaze !== 'undefined' && geoblaze.identify) {
                        try {
                            let pixelValues = await geoblaze.identify(window.georaster, [lon, lat]);
                            
                            // Ensure pixelValues is an array and has at least one value
                            if (Array.isArray(pixelValues) && pixelValues.length > 0) {
                                updateCoordinates(lat, lon, pixelValues[0]);  // Display first band value
                            } else {
                                updateCoordinates(lat, lon);
                            }
                        } catch (error) {
                            console.error("Error getting pixel value:", error);
                            updateCoordinates(lat, lon);
                        }
                    } else {
                        updateCoordinates(lat, lon);
                    }
                });

                var drawnItems = new L.FeatureGroup();
                map.addLayer(drawnItems);
                var drawControl = new L.Control.Draw({
                    edit: { featureGroup: drawnItems },
                    draw: { polygon: true, polyline: true, rectangle: true, circle: false, marker: true }
                });
                map.addControl(drawControl);

                let undoStack = [];
                let redoStack = [];

                map.on(L.Draw.Event.CREATED, function(event) {
                    let layer = event.layer;
                    drawnItems.addLayer(layer);
                    undoStack.push(layer);
                    redoStack = [];
                });

                function undo() {
                    if (undoStack.length > 0) {
                        let lastLayer = undoStack.pop();
                        drawnItems.removeLayer(lastLayer);
                        redoStack.push(lastLayer);
                    }
                }

                function redo() {
                    if (redoStack.length > 0) {
                        let lastLayer = redoStack.pop();
                        drawnItems.addLayer(lastLayer);
                        undoStack.push(lastLayer);
                    }
                }

                function getDrawnShapes() {
                    return JSON.stringify(drawnItems.toGeoJSON());
                }
                function clearMap() {
                    drawnItems.clearLayers(); // Remove all drawn features

                    // Remove all non-tile layers (shapefile layers)
                    map.eachLayer(function(layer) {
                        if (!(layer instanceof L.TileLayer)) {
                            map.removeLayer(layer);
                        }
                    });

                    // Re-add drawnItems to ensure new features can be drawn
                    map.addLayer(drawnItems);

                    // Reinitialize measurement tool
                    map.removeControl(measureControl);
                    measureControl = new L.Control.Measure({
                        primaryLengthUnit: "kilometers",
                        secondaryLengthUnit: "meters",
                        primaryAreaUnit: "sqkilometers",
                        secondaryAreaUnit: "hectares"
                    });
                    map.addControl(measureControl);
                }

                // Distance & Area Measurement Tool
                var measureControl = new L.Control.Measure({
                    primaryLengthUnit: "kilometers",
                    secondaryLengthUnit: "meters",
                    primaryAreaUnit: "sqkilometers",
                    secondaryAreaUnit: "hectares"
                });
                map.addControl(measureControl);
            </script>
        </body>
        </html>
        """

    def load_geotiff(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open GeoTIFF", "", "GeoTIFF Files (*.tif *.tiff)")
        if file_path:
            try:
                #self.tiff_path = os.path.basename(file_path)  # Only store filename
                self.tiff_path = file_path
                with rasterio.open(file_path) as dataset:
                    # Get accurate bounds in EPSG:4326
                    minx, miny, maxx, maxy = transform_bounds(dataset.crs, "EPSG:4326", *dataset.bounds)
                    
                    # Adjust the bounds slightly
                    minx += 0.0005
                    miny += 0.0005
                    maxx -= 0.002
                    maxy -= 0.002

                # JavaScript Code
                js_code = f'''
                (async function() {{
                    if (typeof window.map === 'undefined') {{
                        window.map = L.map('map', {{ crs: L.CRS.EPSG4326 }});
                        L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                            attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
                        }}).addTo(window.map);
                    }}

                    // Load necessary libraries if not already loaded
                    if (typeof window.GeoRasterLayer === 'undefined') {{
                        var scriptGeoRasterLayer = document.createElement("script");
                        scriptGeoRasterLayer.src = "https://unpkg.com/georaster-layer-for-leaflet";
                        scriptGeoRasterLayer.onload = function() {{
                            console.log("GeoRasterLayer loaded successfully");
                        }};
                        document.head.appendChild(scriptGeoRasterLayer);
                    }}

                    if (typeof window.parseGeoraster === 'undefined') {{
                        var scriptGeoRaster = document.createElement("script");
                        scriptGeoRaster.src = "https://unpkg.com/georaster";
                        scriptGeoRaster.onload = function() {{
                            console.log("Georaster loaded successfully");
                        }};
                        document.head.appendChild(scriptGeoRaster);
                    }}

                    if (typeof window.geoblaze === 'undefined') {{
                        var scriptGeoblaze = document.createElement("script");
                        scriptGeoblaze.src = "https://unpkg.com/geoblaze";
                        scriptGeoblaze.onload = function() {{
                            console.log("Geoblaze loaded successfully");
                        }};
                        document.head.appendChild(scriptGeoblaze);
                    }}

                    var tiffUrl = "http://localhost:8000/{self.tiff_path}";  

                    // Wait for parseGeoraster to be available
                    await new Promise(resolve => {{
                        const check = setInterval(() => {{
                            if (typeof parseGeoraster !== 'undefined') {{
                                clearInterval(check);
                                resolve();
                            }}
                        }}, 100);
                    }});

                    fetch(tiffUrl)
                    .then(res => res.arrayBuffer())
                    .then(async arrayBuffer => {{
                        const georaster = await parseGeoraster(arrayBuffer);
                        window.georaster = georaster;

                        var layer = new GeoRasterLayer({{
                            georaster: georaster,
                            opacity: 0.6
                        }}).addTo(window.map);

                        window.map.fitBounds(layer.getBounds());
                    }});
                }})();
                '''

                self.webView.page().runJavaScript(js_code)
                self.statusBar().showMessage(f"GeoTIFF loaded: {file_path}")
            except Exception as e:
                QMessageBox.warning(self, "Error", f"Failed to load GeoTIFF:\n{e}")

    def load_shapefile(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open Shapefile", "", "Shapefiles (*.shp)")
        if file_path:
            self.loaded_shapefile_gdf = gpd.read_file(file_path)
            self.current_shapefile = file_path
            self.current_crs = self.loaded_shapefile_gdf.crs.to_string()
            geojson = self.loaded_shapefile_gdf.to_json()
            js_code = f"""
            var geojsonLayer = L.geoJSON({geojson}, {{
                onEachFeature: function (feature, layer) {{
                    if (feature.geometry.type === "Polygon" || feature.geometry.type === "MultiPolygon") {{
                        var area = L.GeometryUtil.geodesicArea(layer.getLatLngs()[0]) / 1e6; // Convert m² to km²
                        layer.bindPopup("Area: " + area.toFixed(2) + " km²").openPopup();
                    }}
                    else if (feature.geometry.type === "LineString" || feature.geometry.type === "MultiLineString") {{
                        var length = 0;
                        var coords = layer.getLatLngs();
                        
                        // Flatten multi-line strings
                        if (Array.isArray(coords[0])) {{
                            coords = coords.flat();
                        }}

                        for (var i = 0; i < coords.length - 1; i++) {{
                            length += coords[i].distanceTo(coords[i + 1]); // Sum segment lengths
                        }}
                        length = length / 1000; // Convert meters to km
                        layer.bindPopup("Length: " + length.toFixed(2) + " km").openPopup();
                    }}
                }}
            }}).addTo(map);
            """
            self.webView.page().runJavaScript(js_code)
            self.statusBar().showMessage(f"Loaded Shapefile with CRS: {self.current_crs}")

    def clear_map(self):
        self.webView.page().runJavaScript("clearMap();")
        self.current_shapefile = None
        self.statusBar().showMessage("Map cleared.")

    def show_metadata(self):
        if not self.current_shapefile or self.loaded_shapefile_gdf is None or self.loaded_shapefile_gdf.empty:
            QMessageBox.information(self, "Metadata", "No shapefile loaded.")
            return

        try:
            gdf = gpd.read_file(self.current_shapefile)
            metadata = f"CRS: {self.current_crs}\n"
            metadata += f"Number of Features: {len(gdf)}\n"
            metadata += f"Geometry Type: {', '.join(gdf.geometry.type.unique())}\n"
            bounds = gdf.total_bounds  # minx, miny, maxx, maxy
            metadata += f"Bounds: ({bounds[0]}, {bounds[1]}) to ({bounds[2]}, {bounds[3]})\n"

            QMessageBox.information(self, "Shapefile Metadata", metadata)
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to load metadata:\n{e}")

    def save_shapefile(self):
        # Ask user whether to include previously loaded shapefile features
        reply = QMessageBox.question(
            self, "Include Existing Features?",
            "Do you want to include previously loaded shapefile features?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Include both drawn features & loaded shapefile features
            self.webView.page().runJavaScript("getDrawnShapes();", self.process_drawn_shapes_with_existing)
        else:
            # Save only the newly drawn features
            self.webView.page().runJavaScript("getDrawnShapes();", self.process_drawn_shapes)

    def process_drawn_shapes(self, geojson_str):
        try:
            geojson_data = json.loads(geojson_str)
            features = geojson_data.get("features", [])

            valid_geometries = []
            for feature in features:
                try:
                    geom = shape(feature.get("geometry", {}))
                    if geom.is_valid and not geom.is_empty:
                        valid_geometries.append(geom)
                except Exception as e:
                    print(f"Skipping invalid geometry: {e}")

            if not valid_geometries:
                raise ValueError("No valid geometries to save.")

            gdf = gpd.GeoDataFrame(geometry=valid_geometries, crs="EPSG:4326")  # Default

            save_path, _ = QFileDialog.getSaveFileName(self, "Save Shapefile", "", "Shapefiles (*.shp)")
            if save_path:
                if not save_path.endswith(".shp"):
                    save_path += ".shp"

                if self.current_crs != "EPSG:4326":
                    gdf = gdf.to_crs(self.current_crs)

                gdf.to_file(save_path, driver="ESRI Shapefile")
                self.statusBar().showMessage(f"Shapefile saved with CRS: {self.current_crs}")
                print(f"Shapefile saved: {save_path}")

        except Exception as e:
            print(f"Error processing GeoJSON: {e}")

    def process_drawn_shapes_with_existing(self, geojson_str):
        try:
            geojson_data = json.loads(geojson_str)
            features = geojson_data.get("features", [])

            # Separate different types of geometries
            drawn_geometries = {"Polygon": [], "LineString": [], "Point": [], "Circle": []}

            for feature in features:
                try:
                    geom = shape(feature.get("geometry", {}))
                    if geom.is_valid and not geom.is_empty:
                        geom_type = geom.geom_type
                        if geom_type in ["Polygon", "LineString", "Point"]:
                            drawn_geometries[geom_type].append(geom)
                        elif "radius" in feature.get("properties", {}):  # Identify Circles
                            geom = shape(feature["geometry"])
                            radius = feature["properties"]["radius"]
                            drawn_geometries["Circle"].append((geom, radius))  # Store as tuple (geometry, radius)
                except Exception as e:
                    print(f"Skipping invalid geometry: {e}")

            # Convert to GeoDataFrames
            drawn_gdfs = {
                geom_type: gpd.GeoDataFrame(geometry=geometries, crs="EPSG:4326")
                for geom_type, geometries in drawn_geometries.items()
                if geom_type != "Circle" and geometries
            }

            # Convert circles to a GeoDataFrame with radius attribute
            if drawn_geometries["Circle"]:
                circle_data = [{"geometry": geom, "radius": radius} for geom, radius in drawn_geometries["Circle"]]
                drawn_gdfs["Circle"] = gpd.GeoDataFrame(circle_data, crs="EPSG:4326")

            # Check if new features exist
            has_new_features = any(not gdf.empty for gdf in drawn_gdfs.values())
            has_loaded_features = self.loaded_shapefile_gdf is not None and not self.loaded_shapefile_gdf.empty

            # If nothing to save, show a message and return
            if not has_new_features and not has_loaded_features:
                QMessageBox.warning(self, "No Features", "No new features found to save. Not saving.")
                return

            # Merge with existing shapefile data
            if has_loaded_features:
                existing_gdfs = {
                    geom_type: self.loaded_shapefile_gdf[self.loaded_shapefile_gdf.geometry.type == geom_type]
                    for geom_type in drawn_gdfs.keys()
                }

                for geom_type in drawn_gdfs.keys():
                    if geom_type in existing_gdfs:
                        drawn_gdfs[geom_type] = pd.concat([existing_gdfs[geom_type], drawn_gdfs[geom_type]], ignore_index=True)

            # Ask for filename
            save_path, _ = QFileDialog.getSaveFileName(self, "Save Shapefile", "", "Shapefiles (*.shp)")
            if save_path:
                save_base = save_path.rsplit(".", 1)[0]  # Remove extension

                # Save each geometry type separately
                for geom_type, gdf in drawn_gdfs.items():
                    if geom_type == "Circle":
                        gdf.to_file(f"{save_base}_Circles.shp", driver="ESRI Shapefile")
                    else:
                        gdf.to_file(f"{save_base}_{geom_type}.shp", driver="ESRI Shapefile")

                self.statusBar().showMessage(f"Shapefiles saved with CRS: {self.current_crs}")
                print(f"Shapefiles saved: {save_base}_Polygon.shp, {save_base}_LineString.shp, {save_base}_Point.shp, {save_base}_Circles.shp")

        except Exception as e:
            print(f"Error processing GeoJSON: {e}")

    def set_crs(self):
        crs, ok = QInputDialog.getText(self, "Set CRS", "Enter CRS (e.g., EPSG:3857):")
        if ok and crs.strip():
            try:
                # Validate the CRS
                gpd.GeoSeries([shape({"type": "Point", "coordinates": [0, 0]})], crs=crs.strip())

                # Update CRS
                previous_crs = self.current_crs
                self.current_crs = crs.strip()
                self.statusBar().showMessage(f"CRS set to {self.current_crs}")

                # If a shapefile is loaded, reproject it
                if hasattr(self, "gdf") and not self.gdf.empty:
                    self.gdf = self.gdf.to_crs(self.current_crs)

                    # Refresh the map to show the reprojected features
                    geojson = self.gdf.to_json()
                    self.webView.page().runJavaScript("map.eachLayer(function (layer) { if (layer instanceof L.GeoJSON) { map.removeLayer(layer); } });")
                    self.webView.page().runJavaScript(f"L.geoJSON({geojson}).addTo(map);")

            except Exception as e:
                QMessageBox.warning(self, "Invalid CRS", f"Invalid CRS: {crs.strip()}\nError: {e}")

    def toggle_dark_mode(self):
        if self.styleSheet():
            self.setStyleSheet("")
            self.dark_mode_action.setText("Enable Dark Mode")
        else:
            self.setStyleSheet("background-color: black; color: white;")
            self.dark_mode_action.setText("Disable Dark Mode")

    def show_about(self):
        QMessageBox.information(self, "About", """PyGeodit | © Aradhya Chakrabarti 2024-2025
Version: 0.1
PyGeodit is a simple GIS tool for viewing, editing, and analyzing shapefiles produced on OpenStreetMap (OSM) data. It supports annotations, multiple layers, coordinate display, distance and area measurements, shapefile and CRS editing, and more.

License:
This file is part of the Geodit distribution (https://github.com/TimeATronics/geodit).
Copyright (c) 2024-2025 Aradhya Chakrabarti

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
""")

    def show_help(self):
        QMessageBox.information(self, "Help",
                                """PyGeodit - Help
Map Controls:
    1. Pan the Map: Click and drag with the left mouse button.
    2. Zoom In/Out:  
        - Scroll the mouse wheel.
        - Press Ctrl++ to zoom in.
        - Press Ctrl+- to zoom out.

Menu Options:
    1. File Menu: Open and save shapefiles; Open GeoTIFFs, Display metadata, exit.
    2. Edit Menu: Undo/Redo, Set CRS, Clear map.
    3. View Menu: Zoom In/Out, Display theme.

Map Tools:
    1. Measure Distance and area: Click to set points and measure.
    2. Add Annotations: Draw points, lines, or polygons.
    3. Coordinate Display: Shows latitude/longitude when you hover over the map.
    4. Open GeoTIFF: Loads and overlays GeoTIFF file on the corresponding location in the map.
    4. Pixel Value Display: Shows pixel value of GeoTIFF when you hover over the map.
    5. Metadata Panel: Displays CRS, bounds, and feature details of loaded files.

Keyboard Shortcuts:
    - Ctrl + O → Open File
    - Ctrl + S → Save File
    - Ctrl + Z → Undo
    - Ctrl + Y → Redo
    - Ctrl + + → Zoom In
    - Ctrl + - → Zoom Out
    - Ctrl + T → Open GeoTIFF
    - Ctrl + Shift + C → Clear Map
    - F1 → Help 
For more details, check the README.
""")

    def zoom_in(self):
        self.webView.page().runJavaScript("map.zoomIn();")

    def zoom_out(self):
        self.webView.page().runJavaScript("map.zoomOut();")

    def undo_map(self):
        self.webView.page().runJavaScript("undo();")

    def redo_map(self):
        self.webView.page().runJavaScript("redo();")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    splash = QSplashScreen(QPixmap(splash_screen_path))
    splash.show()

    def start_main():
        splash.close()
        global window
        window = MapViewer()
        window.show()

    QTimer.singleShot(3000, start_main)
    sys.exit(app.exec())