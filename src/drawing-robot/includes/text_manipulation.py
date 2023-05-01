import numpy as np
import requests, json, os
import unicodedata

import time

import matplotlib.pyplot as plt
from matplotlib.path import Path
from matplotlib.patches import PathPatch

from fontTools.ttLib import TTFont
from fontTools.pens.recordingPen import RecordingPen
from fontTools.ttLib import TTFont


def scale_vertices(vertices, scale_factor=1.0):
    """
    This function scales the vertices of a polygon by a given scale factor.    
    """   
    
    scaled_vertices = []
    for vertex in vertices:
        scaled_vertices.append((vertex[0] * scale_factor, vertex[1] * scale_factor))
    return scaled_vertices

def download_font(font_name):
    """
    THIS FUNCTION NEED TO BE TESTED AND UPDATED
    This function downloads a font from the Google Fonts API.
    """
    
    url = f'https://www.googleapis.com/webfonts/v1/webfonts?key={api_key}'
    response = requests.get(url)
    fonts = json.loads(response.text)
    # print out the font families
    for font in fonts["items"]:
        print(font["family"])
    font = [font for font in fonts['items'] if font['family'] == font_name][0]
    font_url = font['files']['regular']
    # confirm font url
    print(font_url)
    font_file = requests.get(font_url)
    with open("fonts/" + font_name + '.ttf', 'wb') as f:
        f.write(font_file.content)
    return font_name + '.ttf'


def extract_vertices_codes(commands):
    vertices = []
    codes = []
    points_to_plot = []

    for command, points in commands:
        if command == 'moveTo':
            x, y = points[0]
            points_to_plot.append((x, y, 'ro'))
            vertices.append(points[0])
            codes.append(Path.MOVETO)
        elif command == 'lineTo':
            x, y = points[0]
            points_to_plot.append((x, y, 'ro'))
            vertices.append(points[0])
            codes.append(Path.LINETO)
        elif command == 'qCurveTo':
            num_points = len(points)
            for i in range(0, num_points - 1, 2):
                prev_x, prev_y = vertices[-1]
                x1, y1 = points[i]
                x2, y2 = points[i + 1]
                # Convert quadratic curve to cubic curve
                c1_x = prev_x + 2 / 3 * (x1 - prev_x)
                c1_y = prev_y + 2 / 3 * (y1 - prev_y)
                c2_x = x2 + 1 / 3 * (x1 - x2)
                c2_y = y2 + 1 / 3 * (y1 - y2)

                points_to_plot.append((x1, y1, 'go'))
                points_to_plot.append((x2, y2, 'ro'))
                vertices.extend([(c1_x, c1_y), (c2_x, c2_y), (x2, y2)])
                codes.extend([Path.CURVE4, Path.CURVE4, Path.CURVE4])
        elif command == 'curveTo':
            x1, y1 = points[0]
            x2, y2 = points[1]
            x3, y3 = points[2]
            points_to_plot.append((x1, y1, 'go'))
            points_to_plot.append((x2, y2, 'go'))
            points_to_plot.append((x3, y3, 'ro'))
            vertices.extend(points)
            codes.extend([Path.CURVE4, Path.CURVE4, Path.CURVE4])
        elif command == 'closePath':
            # print("Called closePath")
            vertices.append(vertices[0])
            codes.append(Path.CLOSEPOLY)

    # print(f"Num Verts: {len(vertices)}")

    return vertices, codes, points_to_plot


def extract_glyph_commands_points(ttfont, glyph_name):
    glyph_set = ttfont.getGlyphSet()
    
    if glyph_name not in glyph_set.keys():
        raise ValueError(f'Glyph "{glyph_name}" not found in the font.')
        
    glyph = glyph_set[glyph_name]
    # Use RecordingPen to store the drawing commands
    pen = RecordingPen()
    glyph.draw(pen)

    return pen.value



def get_unicode_digit(char):
    return unicodedata.name(char).lower().split(' ')[-1]

def get_word_commands(word, font, debug=False):
    hmtx_table = font["hmtx"]

    # Initialize the x_offset for the first letter
    x_offset = 0
    word_commands = []
    
    for letter in word:
        if debug:
            print(f'Processing letter: {letter}')
        
        if letter.isdigit():
            letter = get_unicode_digit(letter)
        glyph_id = font.getGlyphID(letter)
        glyph_name = font.getGlyphName(glyph_id)
        # Get glyph commands and points using the extract_glyph_commands_points function
        commands = extract_glyph_commands_points(font, glyph_name)

        # Adjust commands for x_offset
        adjusted_commands = []
        for command, points in commands:
            adjusted_points = [(x + x_offset, y) for x, y in points]
            adjusted_commands.append((command, adjusted_points))
            
        # Get advance width and left side bearing from the horizontal metrics table
        advance_width, left_side_bearing = hmtx_table[glyph_name]

        # Update the x_offset based on the advance width
        x_offset += advance_width
        word_commands.append(adjusted_commands)
    
    return word_commands

def plot_all_points(ax, points_to_plot):
    for x, y, color in points_to_plot:
        pass
        ax.plot(x, y, color)
        time.sleep(5)
        
        plt.close("all")

def render_word(word, font_path, plot_points=False, debug=False, remove_close_path=False):
    fig, ax = plt.subplots()
    font = TTFont(font_path)
    word_commands = get_word_commands(word, font, debug=debug)
    
    for commands in word_commands:
        # vertices, codes, ax = extract_vertices_codes(commands, ax)
        vertices, codes, points_to_plot = extract_vertices_codes(commands)
        if remove_close_path:
            vertices, codes = vertices[:-1], codes[:-1]
        # vertices = scale_vertices(vertices, scale_factor=0.5)
        if debug:
            pass
            print(f'Vertices: {vertices}')
            print(f'Codes: {codes}')
        path = Path(vertices, codes)
        patch = PathPatch(path, edgecolor='blue', facecolor='none', lw=2)
        ax.add_patch(patch)
        if plot_points:
            plot_all_points(ax, points_to_plot)
    
    plt.axis('scaled')
    # plt.show()

def get_coords_word(word, font_path, plot_points=False, debug=False, remove_close_path=False):
    font = TTFont(font_path)
    word_commands = get_word_commands(word, font, debug=debug)
    vertices_word, codes_word, points_to_plot_word = [], [], []
    for commands in word_commands:
        vertices, codes, points_to_plot = extract_vertices_codes(commands)
        if debug:
            print(f'Vertices: {vertices}')
            print(f'Codes: {codes}')
        if remove_close_path:
            vertices_filtered, codes_filtered = [], []
            for i in range(len(codes)):
                if codes[i] != Path.CLOSEPOLY:
                    vertices_filtered.append(vertices[i])
                    codes_filtered.append(codes[i])
        else:
            vertices_filtered, codes_filtered = vertices, codes
        path = Path(vertices_filtered, codes_filtered)
        patch = PathPatch(path, edgecolor='blue', facecolor='none', lw=2)
        vertices_word.extend(vertices_filtered)
        codes_word.extend(codes_filtered)
        points_to_plot_word.extend(points_to_plot)
    
    return vertices_word, codes_word, points_to_plot

def plot_single_glyph(vertices, codes, points_to_plot, ax):
    path = Path(vertices, codes)
    patch = PathPatch(path, edgecolor='blue', facecolor='none', lw=2)
    ax.add_patch(patch)
    plot_all_points(ax, points_to_plot)
    ax.set_aspect('equal')
    plt.axis('scaled')
    # plt.show()

def print_all_glyphs(font_path):
    font = TTFont(font_path)
    glyph_order = font.getGlyphOrder()
    
    for glyph_name in glyph_order:
        print(glyph_name)

def shift_vertices(vertices, shift_x=0, shift_y=0):
    """
    This function shifts the vertices of a polygon by a given shift factor x and y.
    """
    vertices = [(x + shift_x, y + shift_y) for x, y in vertices]
    return vertices

def scale_vertices(vertices, scale_x=1.0, scale_y=1.0):
    """
    This function scales the vertices of a polygon by a given scale factor.    
    """   
    # print(vertices)
    scaled_vertices = []
    for vertex in vertices:
        scaled_vertices.append((vertex[0] * scale_x, vertex[1] * scale_y))
    return scaled_vertices

def rotate_vertices(vertices, angle):
    """
    This function rotates the vertices of a polygon by a given angle.    
    """   
    
    rotated_vertices = []
    for vertex in vertices:
        rotated_vertices.append((vertex[0] * np.cos(angle) - vertex[1] * np.sin(angle), 
                                 vertex[0] * np.sin(angle) + vertex[1] * np.cos(angle)))
    return rotated_vertices



def render(vertices, codes, points_to_plot):
    fig, ax = plt.subplots()
    
    path = Path(vertices, codes)
    patch = PathPatch(path, edgecolor='blue', facecolor='none', lw=2)
    ax.add_patch(patch)
    # if plot_points:
    #     plot_all_points(ax, points_to_plot)
    ax.set_title('Untransformed Points')

    plt.axis('scaled')
    

    return ax.get_xlim(), ax.get_ylim()

