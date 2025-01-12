from bokeh.models import ColumnDataSource, HTMLTemplateFormatter
from bokeh.models.widgets import DataTable, TableColumn
from bokeh.plotting import gridplot

def table_checker(result_list):
  checker_result_keys = set()
  for item in result_list:
    try:
      checker_result_keys.update(item['checker_result'].keys())
    except:
      pass
  table_data = {"bag_name": [], "status": []}
  pass_count = 0
  bag_count = 0
  for key in checker_result_keys:
    table_data[key] = []

  for item in result_list:
    try:
      table_data['bag_name'].append(item['bag_name'])
      checker_result = item['checker_result']
      for key in checker_result_keys:
        value = checker_result.get(key, {})
        table_data[key].append(str(value))
      status = 'passed' if checker_result and all(value.get('success') for value in checker_result.values()) else 'failed'
      table_data['status'].append(status)
      bag_count += 1
      if status == 'passed':
        pass_count += 1
    except:
      pass

  pass_rate = f"{(pass_count / bag_count):.0%}"
  pass_rate_str = f"{pass_count} of {bag_count}"

  formatter = HTMLTemplateFormatter(template='<div title="<%= value %>"><%= value %></div>')

  columns = [TableColumn(field='bag_name', title='bag_name', formatter=formatter)]
  for key in checker_result_keys:
    columns.append(TableColumn(field=key, title=key, formatter=formatter))
  columns.append(TableColumn(field='status', title='status', formatter=formatter))

  table = DataTable(source=ColumnDataSource(data=table_data), columns=columns, width=1200, index_position=-1)

  table_info_data = {"bag_size": [str(bag_count)], "pass_rate": [f"{pass_rate} ({pass_rate_str})"]}
  table_info = DataTable(source=ColumnDataSource(data=table_info_data), columns=[TableColumn(field="bag_size", title="bag_size"), TableColumn(field="pass_rate", title="pass_rate")])

  grid = gridplot([[table], [table_info]])

  return grid