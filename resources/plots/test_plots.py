#  DRAW REEFSCAPE SCORING IN A WEB PAGE
import panel as pn
pn.extension()  # has to come before other matplotlib
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('agg')
import reefscape_utils as ru

# Create persistent figure and axis
fig, ax = plt.subplots(figsize=(6, 9))
plt.close(fig)  # Prevents the initial static display of the figure in Jupyter

def update_plot(event):
    """Update the plot dynamically and trigger Panel update."""
    ru.draw_robot(ax=ax, H=slider_H.value, THETA=slider_THETA.value, time=0, offset=slider_offset.value, static=True)
    interactive_panel.param.trigger('object')  # Force Panel to refresh

# Panel-based interactive UI
slider_H = pn.widgets.FloatSlider(name='H', start=10, end=64, step=0.5, value=30)
slider_THETA = pn.widgets.FloatSlider(name='THETA', start=-15, end=90, step=1, value=45)
slider_offset = pn.widgets.FloatSlider(name='offset', start=-4, end=0, step=0.5, value=-2)
interactive_panel = pn.pane.Matplotlib(fig, tight=True, width=500)

# Add watchers to trigger updates
slider_H.param.watch(update_plot, 'value')
slider_THETA.param.watch(update_plot, 'value')
slider_offset.param.watch(update_plot, 'value')

# Initialize the plot
update_plot(None)
dashboard = pn.Column(interactive_panel, slider_H, slider_THETA, slider_offset)
# dashboard.servable()  # if you want to run in jupyter
pn.serve(dashboard, port=5007, show=True)  # if you want a separate tab
