IDIR =./include
SDIR =./src
CXX=clang++
CFLAGS=-I$(IDIR)

ODIR=obj
LDIR =./lib

LIBS=-ltiff -lglfw -lGL -lpthread -ldl -lrt -lX11 -lGLEW -lgdal

_DEPS = helpers.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = imgui_impl_opengl3.o imgui_impl_glfw.o imgui.o imgui_demo.o imgui_draw.o imgui_tables.o imgui_widgets.o ImGuiFileDialog.o geodit.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: $(SDIR)/%.cpp $(DEPS)
	$(CXX) -c -o $@ $< $(CFLAGS)

geodit: $(OBJ)
	$(CXX) -o $@ $^ $(CFLAGS) -L$(LDIR) $(LIBS)

.PHONY: clean run

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~

run:
	make
	env LIBGL_ALWAYS_SOFTWARE=1 ./geodit
