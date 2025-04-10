"""
GTSAM Copyright 2010-2025, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Author: Porter Zach

This script generates interactive Python notebooks (.ipynb) that document GTSAM 
header files. Since inserting the text of the file directly into the prompt
might be too many tokens, it retrieves the header file content from the GTSAM 
GitHub repository. It then sends it to OpenAI's API for processing, and saves 
the generated documentation as a Jupyter notebook.

Functions:
    is_url_valid(url: str) -> bool:
        Verifies that the supplied URL does not return a 404.

    save_ipynb(text: str, file_path: str) -> str:
        Saves the provided text to a single Markdown cell in a new .ipynb file.

    generate_ipynb(file_path: str, openai_client):
        Generates an interactive Python notebook for the given GTSAM header file 
        by sending a request to OpenAI's API and saving the response.

Usage:
    Run the script with paths to GTSAM header files as arguments. For example:
        python gpt_generate.py gtsam/geometry/Pose3.h
"""

import os
import time
import requests
import argparse
import nbformat as nbf
from openai import OpenAI

_output_folder = "output"
_gtsam_gh_base = "https://raw.githubusercontent.com/borglab/gtsam/refs/heads/develop/"
_asst_id = "asst_na7wYBtXyGU0x5t2RdcnpxzP"
_request_text = "Document the file found at {}."


def is_url_valid(url):
    """Verify that the supplied URL does not return a 404."""
    try:
        response = requests.head(url, allow_redirects=True)
        return response.status_code != 404
    except requests.RequestException:
        return False


def save_ipynb(text: str, file_path: str):
    """Save text to a single Markdown cell in a new .ipynb file."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, _output_folder)
    os.makedirs(output_dir, exist_ok=True)
    output_file = os.path.splitext(os.path.basename(file_path))[0] + ".ipynb"
    output_full_path = os.path.join(output_dir, output_file)

    nb = nbf.v4.new_notebook()
    new_cell = nbf.v4.new_markdown_cell(text)
    nb['cells'].append(new_cell)

    with open(output_full_path, 'w', encoding='utf-8') as file:
        nbf.write(nb, file)

    return output_file


def generate_ipynb(file_path: str, openai_client):
    """Generate an interactive Python notebook for the given GTSAM header file.

    Args:
        file_path (str): The fully-qualified path from the root of the gtsam 
            repository to the header file that will be documented.
        openai_client (openai.OpenAI): The OpenAI client to use.
    """
    # Create the URL to get the header file from.
    url = _gtsam_gh_base + file_path

    if not is_url_valid(url):
        print(f"{url} was not found on the server, or an error occurred.")
        return

    print(f"Sending request to OpenAI to document {url}.")

    # Create a new thread and send the request
    thread = openai_client.beta.threads.create()
    openai_client.beta.threads.messages.create(
        thread_id=thread.id, role="user", content=_request_text.format(url))

    run = openai_client.beta.threads.runs.create(thread_id=thread.id,
                                                 assistant_id=_asst_id)

    print("Waiting for the assistant to process the request...")

    # Wait for request to be processed
    while True:
        run_status = openai_client.beta.threads.runs.retrieve(
            thread_id=thread.id, run_id=run.id)
        if run_status.status == "completed":
            break
        time.sleep(2)

    print("Request processed. Retrieving response...")

    # Fetch messages
    messages = openai_client.beta.threads.messages.list(thread_id=thread.id)
    # Retrieve response text and strip ```markdown ... ```
    text = messages.data[0].content[0].text.value.strip('`').strip('markdown')

    # Write output to file
    output_filename = save_ipynb(text, file_path)

    print(f"Response retrieved. Find output in {output_filename}.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="gpt_generate",
        description=
        "Generates .ipynb documentation files given paths to GTSAM header files."
    )
    parser.add_argument(
        "file_paths",
        nargs='+',
        help=
        "The paths to the header files from the root gtsam directory, e.g. 'gtsam/geometry/Pose3.h'."
    )
    args = parser.parse_args()

    # Retrieves API key from environment variable OPENAI_API_KEY
    client = OpenAI()

    for file_path in args.file_paths:
        generate_ipynb(file_path, client)
