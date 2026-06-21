"use client";

import { useState } from "react";
import { useRouter } from "next/navigation";
import { formatCurrency } from "@/lib/utils";
import { Star, CheckCircle } from "lucide-react";

interface Application {
  id: string;
  providerId: string;
  price: number;
  message: string | null;
  status: string;
  provider: {
    id: string;
    name: string | null;
    image: string | null;
    rating: number | null;
    jobsDone: number | null;
  };
}

interface Task {
  id: string;
  status: string;
  customerId: string;
  budget: number | null;
  payment: { status: string } | null;
  applications: Application[];
}

interface Props {
  task: Task;
  currentUser: { id: string; role: string } | null;
  isCustomer: boolean;
  isProvider: boolean;
  myApplication: { id: string; price: number; status: string } | null;
}

export function TaskActions({ task, currentUser, isCustomer, isProvider, myApplication }: Props) {
  const router = useRouter();
  const [applying, setApplying] = useState(false);
  const [applyForm, setApplyForm] = useState({ price: "", message: "" });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");

  async function submitApplication(e: React.FormEvent) {
    e.preventDefault();
    setLoading(true);
    setError("");

    const res = await fetch("/api/applications", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        taskId: task.id,
        price: parseFloat(applyForm.price),
        message: applyForm.message,
      }),
    });

    if (!res.ok) {
      const data = await res.json();
      setError(data.error || "Failed to apply.");
      setLoading(false);
      return;
    }

    setApplying(false);
    router.refresh();
  }

  async function acceptApplication(applicationId: string) {
    setLoading(true);
    await fetch(`/api/applications/${applicationId}/accept`, { method: "POST" });
    setLoading(false);
    router.refresh();
  }

  async function initiatePayment() {
    setLoading(true);
    const res = await fetch("/api/stripe/checkout", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ taskId: task.id }),
    });
    const data = await res.json();
    if (data.url) window.location.href = data.url;
    setLoading(false);
  }

  const acceptedApp = task.applications.find((a) => a.status === "ACCEPTED");

  return (
    <div className="space-y-6">
      {/* Provider: apply form */}
      {isProvider && task.status === "OPEN" && (
        <div className="bg-white rounded-2xl border border-gray-200 p-6">
          {myApplication ? (
            <div className="text-center py-4">
              <CheckCircle className="h-8 w-8 text-green-500 mx-auto mb-2" />
              <p className="font-semibold text-gray-800">Application submitted</p>
              <p className="text-gray-500 text-sm">
                Your offer: {formatCurrency(myApplication.price)} ·{" "}
                <span className="capitalize">{myApplication.status.toLowerCase()}</span>
              </p>
            </div>
          ) : applying ? (
            <form onSubmit={submitApplication} className="space-y-4">
              <h3 className="font-semibold text-gray-900 text-lg">Submit Your Offer</h3>
              {error && <p className="text-red-500 text-sm">{error}</p>}
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">Your Price ($)</label>
                <input
                  type="number"
                  min="1"
                  step="0.01"
                  required
                  value={applyForm.price}
                  onChange={(e) => setApplyForm((f) => ({ ...f, price: e.target.value }))}
                  className="w-full border border-gray-300 rounded-lg px-4 py-2.5 focus:outline-none focus:ring-2 focus:ring-blue-500"
                  placeholder="e.g. 150"
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">Message (optional)</label>
                <textarea
                  rows={3}
                  value={applyForm.message}
                  onChange={(e) => setApplyForm((f) => ({ ...f, message: e.target.value }))}
                  className="w-full border border-gray-300 rounded-lg px-4 py-2.5 focus:outline-none focus:ring-2 focus:ring-blue-500 resize-none"
                  placeholder="Tell the customer why you're a great fit…"
                />
              </div>
              <div className="flex gap-3">
                <button
                  type="submit"
                  disabled={loading}
                  className="flex-1 bg-blue-600 text-white py-2.5 rounded-lg font-semibold hover:bg-blue-700 disabled:opacity-60"
                >
                  {loading ? "Submitting…" : "Submit Offer"}
                </button>
                <button
                  type="button"
                  onClick={() => setApplying(false)}
                  className="px-4 py-2.5 border border-gray-300 rounded-lg text-gray-700 hover:bg-gray-50"
                >
                  Cancel
                </button>
              </div>
            </form>
          ) : (
            <div className="text-center">
              <h3 className="font-semibold text-gray-900 mb-2">Interested in this task?</h3>
              <p className="text-gray-500 text-sm mb-4">Submit your price and a short message to the customer.</p>
              {currentUser ? (
                <button
                  onClick={() => setApplying(true)}
                  className="bg-blue-600 text-white px-6 py-2.5 rounded-lg font-semibold hover:bg-blue-700"
                >
                  Apply Now
                </button>
              ) : (
                <a href="/login" className="bg-blue-600 text-white px-6 py-2.5 rounded-lg font-semibold hover:bg-blue-700 inline-block">
                  Sign in to Apply
                </a>
              )}
            </div>
          )}
        </div>
      )}

      {/* Customer: payment */}
      {isCustomer && acceptedApp && task.payment?.status !== "PAID" && (
        <div className="bg-green-50 border border-green-200 rounded-2xl p-6">
          <h3 className="font-semibold text-green-800 mb-1">Provider accepted!</h3>
          <p className="text-green-700 text-sm mb-4">
            {acceptedApp.provider.name} will do the job for {formatCurrency(acceptedApp.price)}.
          </p>
          <button
            onClick={initiatePayment}
            disabled={loading}
            className="bg-green-600 text-white px-6 py-2.5 rounded-lg font-semibold hover:bg-green-700 disabled:opacity-60"
          >
            {loading ? "Redirecting…" : `Pay ${formatCurrency(acceptedApp.price)}`}
          </button>
        </div>
      )}

      {task.payment?.status === "PAID" && (
        <div className="bg-blue-50 border border-blue-200 rounded-2xl p-4 text-blue-700 font-medium flex items-center gap-2">
          <CheckCircle className="h-5 w-5" /> Payment received — task in progress!
        </div>
      )}

      {/* Customer: applications list */}
      {isCustomer && task.applications.length > 0 && (
        <div className="bg-white rounded-2xl border border-gray-200 p-6">
          <h3 className="font-semibold text-gray-900 text-lg mb-4">
            Applications ({task.applications.length})
          </h3>
          <div className="space-y-4">
            {task.applications.map((app) => (
              <div
                key={app.id}
                className={`border rounded-xl p-4 ${
                  app.status === "ACCEPTED" ? "border-green-400 bg-green-50" : "border-gray-200"
                }`}
              >
                <div className="flex items-start justify-between gap-3">
                  <div>
                    <p className="font-semibold text-gray-900">{app.provider.name}</p>
                    <div className="flex items-center gap-3 text-sm text-gray-500 mt-0.5">
                      {app.provider.rating != null && (
                        <span className="flex items-center gap-1">
                          <Star className="h-3.5 w-3.5 text-yellow-500" fill="currentColor" />
                          {app.provider.rating.toFixed(1)}
                        </span>
                      )}
                      {app.provider.jobsDone != null && (
                        <span>{app.provider.jobsDone} jobs</span>
                      )}
                    </div>
                    {app.message && (
                      <p className="text-gray-600 text-sm mt-2">{app.message}</p>
                    )}
                  </div>
                  <div className="text-right shrink-0">
                    <p className="font-bold text-gray-900 text-lg">{formatCurrency(app.price)}</p>
                    {app.status === "ACCEPTED" ? (
                      <span className="text-green-600 text-sm font-medium">Accepted</span>
                    ) : app.status === "REJECTED" ? (
                      <span className="text-gray-400 text-sm">Declined</span>
                    ) : task.status === "OPEN" ? (
                      <button
                        onClick={() => acceptApplication(app.id)}
                        disabled={loading}
                        className="mt-2 bg-blue-600 text-white px-4 py-1.5 rounded-lg text-sm font-medium hover:bg-blue-700 disabled:opacity-60"
                      >
                        Accept
                      </button>
                    ) : null}
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Public: no apps yet */}
      {!isCustomer && !isProvider && task.status === "OPEN" && (
        <div className="bg-white rounded-2xl border border-gray-200 p-6 text-center">
          <p className="text-gray-600 mb-4">Sign up as a provider to apply for this task.</p>
          <a
            href="/signup?role=PROVIDER"
            className="bg-blue-600 text-white px-6 py-2.5 rounded-lg font-semibold hover:bg-blue-700 inline-block"
          >
            Become a Provider
          </a>
        </div>
      )}
    </div>
  );
}
