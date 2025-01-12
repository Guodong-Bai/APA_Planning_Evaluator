from bokeh.plotting import figure, show
from bokeh.layouts import gridplot
from bokeh.models import ColumnDataSource, NumeralTickFormatter, TableColumn, DataTable, WheelZoomTool, PanTool, Range1d, FixedTicker
import numpy as np
from bokeh.io import output_file
from bokeh.layouts import layout, column, row

def create_histogram(data, title, color, is_use_abs = False):
  title = title + ", bag_size =" + str(len(data))
  fig = figure(title=title, width=400, height=400, tools="hover", tooltips=[("Nums", "@hist"), ("Percentage", "@hist_percentage{:.1%}")])

  # Calculate the histogram data
  data_ = data.copy()
  data_.sort()
  # hist, edges = np.histogram(data_, bins=10)
  hist, edges = np.histogram(data_, bins = np.linspace(min(data), max(data), num=10))
  total_count = len(data_)
  hist_percentage = hist / total_count

  source = ColumnDataSource(data=dict(hist=hist, hist_percentage=hist_percentage, left=edges[:-1], right=edges[1:]))

  # Set y-axis to display as percentage
  fig.y_range = Range1d(0, 1)
  fig.yaxis[0].formatter = NumeralTickFormatter(format="0%")
  fig.yaxis[0].ticker = FixedTicker(ticks=[0, 0.2, 0.4, 0.6, 0.8, 1])

  # Draw a histogram
  fig.quad(top='hist_percentage', bottom=0, left='left', right='right', fill_color=color, line_color="white", alpha=0.5, source=source)

  # Draw table
  max_value = round(max(data_), 2)
  min_value = round(min(data_), 2)
  avg_value = round(sum(data_) / len(data_), 2)
  table_source = ColumnDataSource(data={"name":["Max Value", "Min Value", "Avg Value"], "data":[max_value, min_value, avg_value]})

  if is_use_abs:
    data_abs = data_.copy()
    for i in range(len(data_abs)):
      if data_abs[i] < 0.0:
        data_abs[i] *= -1.0

    max_abs_value = round(max(data_abs), 2)
    min_abs_value = round(min(data_abs), 2)
    avg_abs_value = round(sum(data_abs) / len(data_abs), 2)
    table_source = ColumnDataSource(data={"name":["Max Value", "Min Value", "Avg Value", "max_abs_value", "min_abs_value", "avg_abs_value"], "data":[max_value, min_value, avg_value, max_abs_value, min_abs_value, avg_abs_value]})

  columns = [
    TableColumn(field="name", title="Name"),
    TableColumn(field="data", title="Data")
    ]
  data_table = DataTable(source=table_source, columns=columns, width=300, height=200)

  fig.add_tools(WheelZoomTool())
  fig.add_tools(PanTool())
  # fig.toolbar.active_scroll = fig.select_one({'type': WheelZoomTool})
  # fig.legend.click_policy = 'hide'
  fig = column(fig, data_table)

  return fig

def create_histogram_abs(data, title, color):
  title = title + ", bag_size =" + str(len(data))
  fig = figure(title=title, width=400, height=400, tools="hover", tooltips=[("Nums", "@hist"), ("Percentage", "@hist_percentage{:.1%}")])

  # Calculate the histogram data
  data_ = data.copy()
  data_.sort()
  # hist, edges = np.histogram(data_, bins=10)
  hist, edges = np.histogram(data_, bins = np.linspace(min(data), max(data), num=10))
  total_count = len(data_)
  hist_percentage = hist / total_count

  source = ColumnDataSource(data=dict(hist=hist, hist_percentage=hist_percentage, left=edges[:-1], right=edges[1:]))

  # Set y-axis to display as percentage
  fig.y_range = Range1d(0, 1)
  fig.yaxis[0].formatter = NumeralTickFormatter(format="0%")
  fig.yaxis[0].ticker = FixedTicker(ticks=[0, 0.2, 0.4, 0.6, 0.8, 1])

  # Draw a histogram
  fig.quad(top='hist_percentage', bottom=0, left='left', right='right', fill_color=color, line_color="white", alpha=0.5, source=source)

  # Draw table
  data_abs = data_.copy()
  for i in range(len(data_abs)):
    if data_abs[i] < 0.0:
      data_abs[i] *= -1.0

  max_abs_value = round(max(data_abs), 2)
  min_abs_value = round(min(data_abs), 2)
  avg_abs_value = round(sum(data_abs) / len(data_abs), 2)
  table_source = ColumnDataSource(data={"name":["max_abs_value", "min_abs_value", "avg_abs_value"], "data":[max_abs_value, min_abs_value, avg_abs_value]})
  columns = [
    TableColumn(field="name", title="Name"),
    TableColumn(field="data", title="Data")
    ]
  data_table = DataTable(source=table_source, columns=columns, width=300, height=200)

  fig.add_tools(WheelZoomTool())
  fig.add_tools(PanTool())
  # fig.toolbar.active_scroll = fig.select_one({'type': WheelZoomTool})
  # fig.legend.click_policy = 'hide'
  fig = column(fig, data_table)

  return fig

def create_histogram_flli9(data_dict,title, color):
  data_ = list(data_dict.values())
  title = title + ", bag_size =" + str(len(data_))
  fig = figure(title=title, width=460, height=400, tools="hover", tooltips=[("Nums", "@hist"), ("Percentage", "@hist_percentage{:.1%}")])

  # Calculate the histogram data

  # data_ = data.copy()

  data_.sort()
  # hist, edges = np.histogram(data_, bins=10)
  hist, edges = np.histogram(data_, bins = np.linspace(min(data_), max(data_), num=10))
  total_count = len(data_)
  hist_percentage = hist / total_count

  source = ColumnDataSource(data=dict(hist=hist, hist_percentage=hist_percentage, left=edges[:-1], right=edges[1:]))

  # Set y-axis to display as percentage
  fig.y_range = Range1d(0, 1)
  fig.yaxis[0].formatter = NumeralTickFormatter(format="0%")
  fig.yaxis[0].ticker = FixedTicker(ticks=[0, 0.2, 0.4, 0.6, 0.8, 1])

  # Draw a histogram
  fig.quad(top='hist_percentage', bottom=0, left='left', right='right', fill_color=color, line_color="white", alpha=0.5, source=source)

  # Draw table
  max_value = round(max(data_), 2)
  min_value = round(min(data_), 2)
  for i in range(len(data_)):
    data_[i] = abs(data_[i])
  avg_value = round(sum(data_) / len(data_), 2)
  data = {"name":(["Max Value", "Min Value", "Avg Value"] + list(data_dict.keys())), "data":([max_value, min_value, avg_value] + list(data_dict.values()))}
  table_source = ColumnDataSource(data)
  columns = [
    TableColumn(field="name", title="Name"),
    TableColumn(field="data", title="Data")
    ]
  data_table = DataTable(source=table_source, columns=columns, width=460, height=150)

  fig.add_tools(WheelZoomTool())
  fig.add_tools(PanTool())
  # fig.toolbar.active_scroll = fig.select_one({'type': WheelZoomTool})
  # fig.legend.click_policy = 'hide'
  fig = column(fig, data_table)

  return fig