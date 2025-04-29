import tkinter as tk
from tkinter import filedialog
import pandas as pd
import plotly.graph_objs as go


class CsvPlotter:

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("CSV Data Visualization")
        self.root.minsize(600, 300)  # Set minimum window size

        # Label to display currently loaded CSV file
        self.loaded_csv_label = tk.Label(self.root,
                                         text="No CSV file loaded",
                                         wraplength=380)
        self.loaded_csv_label.pack()

        # Button to load CSV
        self.load_button = tk.Button(self.root,
                                     text="Load CSV",
                                     command=self.load_csv)
        self.load_button.pack(pady=5)

        # Frame for X label and dropdown
        self.x_frame = tk.Frame(self.root)
        self.x_frame.pack(pady=5, fill="both", expand=True)

        # X label and dropdown
        self.x_label = tk.Label(self.x_frame, text="X Axis:")
        self.x_label.pack(side="left",
                          padx=(5, 0))  # Padding only on the left side
        self.x_dropdown = tk.StringVar(self.root)
        self.x_dropdown_menu = tk.OptionMenu(self.x_frame, self.x_dropdown, "")
        self.x_dropdown_menu.pack(side="left",
                                  padx=(5, 0))  # Padding only on the left side

        # Frame for Y label and listboxes
        self.y_frame = tk.Frame(self.root)
        self.y_frame.pack(pady=5, fill="both", expand=True)

        # Y label
        self.y_label = tk.Label(self.y_frame, text="Y Axis:")
        self.y_label.pack(side="left",
                          padx=(5, 0))  # Padding only on the left side

        # Frame for selected listbox and search
        self.selected_frame = tk.Frame(self.y_frame)
        self.selected_frame.pack(side="left", fill="both", expand=True, padx=5)

        # Label for selected listbox
        self.selected_label = tk.Label(self.selected_frame,
                                       text="Selected to plot:",
                                       anchor="w")
        self.selected_label.pack(fill="x")

        # Search entry for selected listbox
        self.selected_search_entry = tk.Entry(self.selected_frame)
        self.selected_search_entry.pack(fill="x")
        self.selected_search_entry.bind("<KeyRelease>",
                                        self.search_selected_listbox)

        # Selected listbox
        self.selected_listbox = tk.Listbox(self.selected_frame,
                                           selectmode="multiple",
                                           exportselection=False,
                                           height=10)
        self.selected_listbox.pack(side="top", fill="both", expand=True)
        self.selected_listbox.bind("<Double-Button-1>",
                                   self.remove_from_selected)

        # Remove button for selected listbox
        self.remove_button = tk.Button(self.selected_frame,
                                       text="Remove -->",
                                       command=self.remove_from_selected)
        self.remove_button.pack(side="bottom", fill="x")

        # Frame for non-selected listbox and search
        self.non_selected_frame = tk.Frame(self.y_frame)
        self.non_selected_frame.pack(side="left",
                                     fill="both",
                                     expand=True,
                                     padx=5)

        # Label for non-selected listbox
        self.non_selected_label = tk.Label(self.non_selected_frame,
                                           text="Non-selected:",
                                           anchor="w")
        self.non_selected_label.pack(fill="x")

        # Search entry for non-selected listbox
        self.non_selected_search_entry = tk.Entry(self.non_selected_frame)
        self.non_selected_search_entry.pack(fill="x")
        self.non_selected_search_entry.bind("<KeyRelease>",
                                            self.search_non_selected_listbox)

        # Non-selected listbox
        self.non_selected_listbox = tk.Listbox(
            self.non_selected_frame,
            selectmode="multiple",
            exportselection=False,
            height=5,
        )
        self.non_selected_listbox.pack(side="top", fill="both", expand=True)
        self.non_selected_listbox.bind("<Double-Button-1>",
                                       self.add_to_selected)

        # Add button for selected listbox
        self.add_button = tk.Button(
            self.non_selected_frame,
            text="<-- Add to plot",
            command=self.add_to_selected,
        )
        self.add_button.pack(side="bottom", fill="x")

        # Title and Button to plot graph
        self.title_button_frame = tk.Frame(self.root)
        self.title_button_frame.pack(pady=15)

        self.title_label = tk.Label(self.title_button_frame,
                                    text="Plot Title:")
        self.title_label.pack(side="left", padx=(5, 0))
        self.plot_title_entry = tk.Entry(self.title_button_frame)
        self.plot_title_entry.pack(side="left")

        self.plot_button = tk.Button(
            self.title_button_frame,
            text="Click to Plot",
            command=self.plot_graph,
        )
        self.plot_button.pack(side='left', padx=10)

        self.root.mainloop()

    def load_csv(self):
        file_path = filedialog.askopenfilename(filetypes=[("CSV files",
                                                           "*.csv")])
        if file_path:
            try:
                self.df = pd.read_csv(file_path)
                del self.df[" "]
                self.df = self.df.rename(columns=lambda x: x.strip())
                self.unique_headers = set(
                    col.split("__")[0] for col in self.df.columns)
                self.loaded_csv_label.config(
                    text=f"Loaded CSV file:\n{file_path}")
                self.update_dropdowns()
            except Exception as e:
                print("Error loading CSV:", e)

    def update_dropdowns(self):
        x_columns = self.df.columns.tolist()
        y_columns = sorted(self.unique_headers)  # Sort columns alphabetically
        self.x_dropdown.set("")  # Reset dropdown
        self.x_dropdown_menu["menu"].delete(0, "end")
        self.selected_listbox.delete(0, "end")  # Reset selected listbox
        self.non_selected_listbox.delete(0,
                                         "end")  # Reset non-selected listbox
        for col in x_columns:
            self.x_dropdown_menu["menu"].add_command(label=col,
                                                     command=tk._setit(
                                                         self.x_dropdown, col))
        if 'time' in x_columns:
            self.x_dropdown.set("time")
        for col in y_columns:
            self.non_selected_listbox.insert("end", col)

    def add_to_selected(self, event=None):
        selected_items = self.non_selected_listbox.curselection()
        for idx in selected_items[::
                                  -1]:  # Reverse the selection to maintain order when moving
            item = self.non_selected_listbox.get(idx)
            self.selected_listbox.insert("end", item)
            self.non_selected_listbox.delete(idx)
        self.sort_listbox(self.selected_listbox)

    def remove_from_selected(self, event=None):
        selected_items = self.selected_listbox.curselection()
        for idx in selected_items[::
                                  -1]:  # Reverse the selection to maintain order when moving
            item = self.selected_listbox.get(idx)
            self.non_selected_listbox.insert("end", item)
            self.selected_listbox.delete(idx)
        self.sort_listbox(self.non_selected_listbox)

    def sort_listbox(self, listbox):
        items = list(listbox.get(0, "end"))
        items.sort()
        listbox.delete(0, "end")
        for item in items:
            listbox.insert("end", item)

    def search_selected_listbox(self, event=None):
        self.search_listbox(self.selected_search_entry, self.selected_listbox)

    def search_non_selected_listbox(self, event=None):
        self.search_listbox(self.non_selected_search_entry,
                            self.non_selected_listbox)

    def search_listbox(self, search_entry, listbox):
        search_query = search_entry.get().lower()
        listbox.delete(0, "end")
        for item in sorted(self.unique_headers):
            if search_query in item.lower():
                listbox.insert("end", item)

    def plot_graph(self):
        x_column = self.x_dropdown.get()
        selected_cols = self.selected_listbox.get(0, "end")

        y_columns = []
        for header_name in selected_cols:
            y_columns += [
                col for col in self.df.columns
                if col.startswith(header_name + "__")
            ]
            y_columns += [col for col in self.df.columns if col == header_name]

        if x_column and y_columns:
            # Create a list to hold traces
            traces = []

            # Iterate through selected Y-columns
            for y_column in y_columns:
                # Create trace for each Y-column
                trace = go.Scatter(
                    x=self.df[x_column],
                    y=self.df[y_column],
                    mode="lines",
                    name=y_column,
                )
                traces.append(trace)

            # Create layout
            layout = go.Layout(
                title=self.plot_title_entry.get()
                if self.plot_title_entry.get() else "Sai CSV Plot",
                xaxis=dict(title=x_column),
                # yaxis=dict(title="Value"),
            )

            # Create figure
            fig = go.Figure(data=traces, layout=layout)
            fig.update_layout(title=layout.title)

            # Show plot
            fig.show()

    def run(self):
        self.root.mainloop()


# Example usage:
if __name__ == "__main__":
    CsvPlotter()
