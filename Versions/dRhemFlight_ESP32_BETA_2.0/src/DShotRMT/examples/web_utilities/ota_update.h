/**
 * @file ota_update.h
 * @brief Contains the HTML, CSS, and JavaScript for the OTA (Over-The-Air) update web page.
 * @author Wastl Kraus
 * @date 2025-10-04
 * @license MIT
 */

#pragma once

// OTA Update HTML
const char *ota_html = R"rawliteral(
<!DOCTYPE html>
<html>

<head>
    <title>OTA Update - DShotRMT</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, Helvetica, sans-serif;
            background-color: #2c3e50;
            color: #ecf0f1;
            margin: 0;
            padding: 20px;
        }

        .container {
            max-width: 600px;
            margin: 0 auto;
            background-color: #34495e;
            padding: 30px;
            border-radius: 12px;
            box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
        }

        h1 {
            text-align: center;
            color: #3498db;
        }

        .form-group {
            margin-bottom: 20px;
        }

        label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
        }

        input[type="file"] {
            width: 100%;
            padding: 8px;
            border: 1px solid #7f8c8d;
            border-radius: 4px;
            background-color: #2c3e50;
            color: #ecf0f1;
        }

        .btn {
            background-color: #3498db;
            color: white;
            padding: 12px 20px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            width: 100%;
            font-size: 16px;
        }

        .btn:hover {
            background-color: #2980b9;
        }

        .btn:disabled {
            background-color: #7f8c8d;
            cursor: not-allowed;
        }

        .progress {
            width: 100%;
            background-color: #2c3e50;
            border-radius: 4px;
            margin-top: 10px;
            display: none;
        }

        .progress-bar {
            width: 0%;
            height: 30px;
            background-color: #27ae60;
            border-radius: 4px;
            text-align: center;
            line-height: 30px;
            color: white;
        }

        .message {
            margin-top: 20px;
            padding: 10px;
            border-radius: 4px;
            display: none;
        }

        .success {
            background-color: #27ae60;
        }

        .error {
            background-color: #e74c3c;
        }

        .warning {
            background-color: #f39c12;
            color: #2c3e50;
            margin-bottom: 20px;
            padding: 15px;
            border-radius: 8px;
            font-weight: bold;
        }

        .back-link {
            text-align: center;
            margin-top: 20px;
        }

        .back-link a {
            color: #3498db;
            text-decoration: none;
        }

        .back-link a:hover {
            text-decoration: underline;
        }
    </style>
</head>

<body>
    <div class="container">
        <h1>OTA Firmware Update</h1>

        <div class="warning">
            WARNING: Stop motors before starting update!
        </div>

        <form id="upload_form" enctype="multipart/form-data">
            <div class="form-group">
                <label for="update">Firmware File (.bin):</label>
                <input type="file" id="update" name="update" accept=".bin" required>
            </div>
            <button type="submit" class="btn" id="uploadBtn">Update!</button>
        </form>

        <div class="progress" id="progressDiv">
            <div class="progress-bar" id="progressBar">0%</div>
        </div>

        <div class="message" id="message"></div>

    </div>

    <script>
        document.getElementById('upload_form').addEventListener('submit', function (e) {
            e.preventDefault();

            const fileInput = document.getElementById('update');
            const file = fileInput.files[0];

            if (!file) {
                showMessage('Choose a valid update file.', 'error');
                return;
            }

            if (!file.name.endsWith('.bin')) {
                showMessage('Choose a valid update file.', 'error');
                return;
            }

            uploadFile(file);
        });

        function uploadFile(file) {
            const formData = new FormData();
            formData.append('update', file);

            const xhr = new XMLHttpRequest();

            // Progress tracking
            xhr.upload.addEventListener('progress', function (e) {
                if (e.lengthComputable) {
                    const percentComplete = (e.loaded / e.total) * 100;
                    updateProgress(percentComplete);
                }
            });

            // Upload complete
            xhr.addEventListener('load', function () {
                if (xhr.status === 200) {
                    showMessage('Update successfull! Restarting...', 'success');
                    setTimeout(() => {
                        window.location.href = '/';
                    }, 5000);
                } else {
                    showMessage('Update failed: ' + xhr.responseText, 'error');
                    resetUpload();
                }
            });

            // Upload error
            xhr.addEventListener('error', function () {
                showMessage('Connection error during Update.', 'error');
                resetUpload();
            });

            // Start upload
            document.getElementById('uploadBtn').disabled = true;
            document.getElementById('uploadBtn').textContent = 'buffering...';
            document.getElementById('progressDiv').style.display = 'block';

            xhr.open('POST', '/update');
            xhr.send(formData);
        }

        function updateProgress(percent) {
            const progressBar = document.getElementById('progressBar');
            progressBar.style.width = percent + '%';
            progressBar.textContent = Math.round(percent) + '%';
        }

        function showMessage(text, type) {
            const messageDiv = document.getElementById('message');
            messageDiv.textContent = text;
            messageDiv.className = 'message ' + type;
            messageDiv.style.display = 'block';
        }

        function resetUpload() {
            document.getElementById('uploadBtn').disabled = false;
            document.getElementById('uploadBtn').textContent = 'Update!';
            document.getElementById('progressDiv').style.display = 'none';
            document.getElementById('progressBar').style.width = '0%';
            document.getElementById('progressBar').textContent = '0%';
        }
    </script>
</body>

</html>
)rawliteral";
