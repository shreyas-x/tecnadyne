console.log("Started");

const socket = new WebSocket("ws://localhost:8000");

socket.addEventListener('open', (e) => {
	socket.send("Connection Established");
});

socket.addEventListener('message', (e) => {
	console.log(e.data);
});

button = document.getElementById("start");
button.onclick = () => {
	socket.send("start,1,2,3");
}

heartbeat = setInterval(() => {
	socket.send("status,1,2,3");
}, 1000); 
