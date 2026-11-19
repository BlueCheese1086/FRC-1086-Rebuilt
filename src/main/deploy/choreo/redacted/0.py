import os
import shutil

def move_files_by_keywords(keywords):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(current_dir)
    search_terms = [term.strip().lower() for term in keywords]
    for filename in os.listdir(parent_dir):
        if any(term in filename.lower() for term in search_terms):
            source_path = os.path.join(parent_dir, filename)
            if os.path.isfile(source_path) and filename != os.path.basename(__file__):
                destination_path = os.path.join(current_dir, filename)
                try:
                    shutil.move(source_path, destination_path)
                    print(f"Moved: {filename}")
                    count += 1
                except Exception as e:
                    print(f"Error moving {filename}: {e}")

if __name__ == "__main__":
    move_files_by_keywords([])

#dont worry about this