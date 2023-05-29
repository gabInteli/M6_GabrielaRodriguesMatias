# Aqui foi desenvolvido o backend para a rota de envio de imagens para a detecção de rachaduras por meio do YoloV8 
# Para executar o servidor, basta executar o comando "python main.py" no terminal

# Importa bibliotecas
import fastapi
import uvicorn 
import pydantic
from fastapi.middleware.cors import CORSMiddleware

# Cria o servidor
app = fastapi.FastAPI()

from yolo import get_yolo_results

# Define as origens permitidas para o servidor
origins = [
    "http://localhost",
    "http://localhost:3000",
]

# Define as configurações do CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.post("/upload-image")
async def upload_image(image: bytes = fastapi.File(...)): 
    print('bati')
    with open("uploaded_image.jpg", "wb") as file:
        file.write(image)
    print(get_yolo_results("uploaded_image.jpg"))

    return {"message": "Image uploaded successfully"}

# Executa o servidor
if __name__ == "__main__":
    uvicorn.run(app)