from text_manipulation import *

# ============================ globals ============================== 

alphabetsCap = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
alphabetsSmall = 'abcdefghijklmnopqrstuvwxyz'
numbers = '0123456789'
specialCharacters = '!@#$%^&*()_+-=[]{}|;:,./<>?'

# API key of Madhu for the Google Fonts Developer API
api_key = 'AIzaSyCheZLi8YDZ9_nbCQ8kVDTEHG3MFpSQu9M'

# Set the API endpoint URL
url = f'https://www.googleapis.com/webfonts/v1/webfonts?key={api_key}&sort=popularity'

fonts_path = 'fonts/'

# file_name = download_font("Roboto")
# file_name = download_font("Roboto Condensed")
# file_name = download_font("Sura")
# file_name = download_font("PT Serif")
# file_name = download_font("Roboto Serif")

# Load the TrueType font file
# font = TTFont(fonts_path + file_name)

# glyphs = font.getGlyphOrder()
# glyph_names = font.getGlyphNames()

# ============================ globals ============================== 

# font_path = fonts_path + 'Stick-Regular.ttf'
font_path = fonts_path + 'ReliefSingleLine-Regular.ttf'
# font_path = fonts_path + "Roboto.ttf"
# font_path = fonts_path + "Roboto Serif.ttf"
# font_path = fonts_path + "Roboto Condensed.ttf"
# font_path = fonts_path + "Sura.ttf"
# font_path = fonts_path + "PT Serif.ttf"
# font_path = 'fonts/EMS_Allure_Smooth/EMS_Allure_Smooth.ttf'
word = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
# render_word(word, font_path)
# render_word("AB", font_path, plot_points=True, debug=False, remove_close_path=True)
# render_word("NOPQRSTUVWXYZ", font_path, plot_points=False, debug=False, remove_close_path=True)

vertices, codes, points_to_plot = get_coords_word("C", font_path, plot_points=True, debug=False, remove_close_path=True)
render(vertices, codes, points_to_plot)
print("vertices: ", vertices)
scale_x, scale_y = 1/2909, 1/3200

scale_x, scale_y = 1/2909, 1/3200
vertices_scaled = scale_vertices(vertices, scale_x, scale_y)
vertices_shifted = shift_vertices(vertices_scaled, shift_x= -0.1788297, shift_y=0.378)
print("vertices_scaled: ", vertices_scaled)
print("vertices_shifted: ", vertices_shifted)
flipped_vertices = [(y, x) for x, y in vertices_shifted]
print("flipped_vertices: ", flipped_vertices)
print("codes: ", codes)
print("points_to_plot: ", points_to_plot)

render_word("AB", font_path, plot_points=True, remove_close_path=True)
