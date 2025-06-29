/*
This file handles the inquiry page by allowing users to input inquiries and interact (reply and view) with other inquiries 
through the use of a database.
Users are also able to delete their own inquires. Also, filtering is done to sensor inquiries and replies.

Note: This is one of many JavaScript files used for the website.
*/

document.addEventListener("DOMContentLoaded", () => {
  const form = document.getElementById("new-inquiry-form");
  const inquiryList = document.getElementById("inquiry-list");

  if (!form || !inquiryList) {
    console.error(" form or inquiryList not found!");
    return;
  }

  let inquiries = [];
  let currentUser = null;

fetch("/api/user/me", { credentials: "same-origin" })
  .then(r => {
    if (!r.ok) throw new Error("User fetch failed with status " + r.status);
    return r.json();
  })
  .then(user => {
    console.log("Logged in user:", user); 
    if (!user) return alert("You must be logged in.");
    currentUser = user;

    return fetch("/api/inquiries")
      .then(r => {
        if (!r.ok) throw new Error("Inquiries fetch failed with status " + r.status);
        return r.json();
      })
      .then(data => {
        console.log(" Got inquiries from backend:", data);
        inquiries = data;
        data.forEach((inquiry, idx) =>
          addInquiryToDOM(inquiry, idx, currentUser, true)
        );
      });
  })
  .catch(err => {
    console.error("Error during fetch chain:", err);
  });

  form.addEventListener("submit", e => {
    e.preventDefault();
    if (!currentUser) return alert("Please log in first.");

    const question = document.getElementById("question").value.trim();
    if (!question) return;

        fetch("/api/inquiries", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
            userName: `${currentUser.firstName} ${currentUser.lastName}`, // send full name as one field
            inquiry: question
        })
        })
        .then(r => r.json())
        .then(newInquiry => {
        // addInquiryToDOM(newInquiry, inquiries.length, currentUser);
        addInquiryToDOM(newInquiry, inquiries.length, currentUser, true);
        inquiries.push(newInquiry);
        form.reset();
        console.log("YESS POSTED");
        });
        });

  function addInquiryToDOM(inquiry, index, currentUser, prepend = false) {
    const div = document.createElement("div");
    div.className = "inquiry";

    div.innerHTML = `
      <p><strong>From:</strong> ${inquiry.userName}</p>
      <p><strong>Question:</strong> ${inquiry.Inquiry}</p>
      <p class="timestamp"><em>${inquiry.TimeSubmitted || ""}</em></p>
      <button class="reply-btn">Reply</button>
      <div class="reply-section" style="display: none; margin-top: 10px;">
        <textarea class="reply-text" placeholder="Your reply..."></textarea>
        <button class="send-reply">Send</button>
      </div>
      <div class="reply-display" style="margin-top: 10px;"></div>
    `;

    if ((currentUser?.StudentID === inquiry.author_id) || currentUser?.isAdmin) {
      const deleteBtn = document.createElement("button");
      deleteBtn.className = "delete-inquiry";
      deleteBtn.style = "margin-top: 5px; color:red;";
      deleteBtn.textContent = "Delete Inquiry";
      deleteBtn.addEventListener("click", () => {
        if (confirm("Delete this inquiry?")) {
            fetch(`/api/inquiries/${inquiry.inquiryID}`, {
                method: "DELETE"
            })
            .then(r => {
                if (r.ok) {
                div.remove();
                } else {
                alert("Failed to delete inquiry.");
                }
            });
        }
      });
      div.appendChild(deleteBtn);
    }

    const replyBtn = div.querySelector(".reply-btn");
    const replySection = div.querySelector(".reply-section");
    const sendBtn = div.querySelector(".send-reply");
    const replyBox = div.querySelector(".reply-text");
    const display = div.querySelector(".reply-display");

    if (inquiry.replies?.length) {
      inquiry.replies.forEach((rep) => {
        const replyID = rep.id;
        const replyContainer = document.createElement("div");
        replyContainer.className = "reply-item";

        const replyText = document.createElement("p");
        replyText.innerHTML = `<strong>Reply:</strong> ${rep.reply} <em style="font-size: 0.8em;">${rep.timeReplied}</em>`;
        
        replyText.innerHTML = ` <strong>${rep.userName}:</strong> ${rep.reply}
            <em style="font-size: 0.8em;">${rep.timeReplied}</em>`;

        replyContainer.appendChild(replyText);

        if ((currentUser?.StudentID === rep.author_id) || currentUser?.isAdmin) {
          const deleteReplyBtn = document.createElement("button");
          deleteReplyBtn.textContent = "Delete Reply";
          deleteReplyBtn.style.marginLeft = "10px";
          deleteReplyBtn.style.color = "red";
          deleteReplyBtn.addEventListener("click", () => {
            if (confirm("Delete this reply?")) {
                fetch(`/api/replies/${rep.replyID}`, { method: "DELETE" });
              replyContainer.remove();
            }
          });
          replyContainer.appendChild(deleteReplyBtn);
        }

        display.appendChild(replyContainer);
      });
    }

    replyBtn.addEventListener("click", () => {
      replySection.style.display = replySection.style.display === "none" ? "block" : "none";
    });

    sendBtn.addEventListener("click", () => {
      const text = replyBox.value.trim();
      if (!text) return;

      const reply = {
        reply: text,
        timeReplied: new Date().toISOString(),
        userName: `${currentUser.firstName} ${currentUser.lastName}`,
        inquiryID: inquiry.inquiryID
      };

      fetch("/api/replies", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(reply)
      })
      .then(r => r.json())
      .then(savedReply => {
        replyBox.value = "";
        replySection.style.display = "none";

        const replyContainer = document.createElement("div");
        replyContainer.className = "reply-item";

        const replyText = document.createElement("p");
        
        replyText.innerHTML = `
            <strong>${savedReply.userName}:</strong> ${savedReply.reply}
            <em style="font-size: 0.8em;">${savedReply.timeReplied}</em>`;
        replyContainer.appendChild(replyText);

        if (currentUser?.StudentID === savedReply.author_id || currentUser?.isAdmin) {
          const deleteReplyBtn = document.createElement("button");
          deleteReplyBtn.textContent = "Delete Reply";
          deleteReplyBtn.style.marginLeft = "10px";
          deleteReplyBtn.style.color = "red";
          deleteReplyBtn.addEventListener("click", () => {
            if (confirm("Delete this reply?")) {
              replyContainer.remove();
            }
          });
          replyContainer.appendChild(deleteReplyBtn);
        }

        display.appendChild(replyContainer);
      });
    });

    // inquiryList.appendChild(div);
    if (prepend) {
        inquiryList.prepend(div);
        } else {
        inquiryList.appendChild(div);
        }
            
  }
});